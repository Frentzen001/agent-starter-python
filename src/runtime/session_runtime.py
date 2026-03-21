from __future__ import annotations

import asyncio
import logging
import math
import os
from dataclasses import dataclass

import httpx
from pathlib import Path
from typing import TYPE_CHECKING, Any, Mapping

import yaml
from livekit import rtc
from livekit.agents import AgentSession, JobContext, inference, room_io
from livekit.plugins import noise_cancellation
from livekit.plugins.turn_detector.multilingual import MultilingualModel

from domain.interaction_domain import InteractionDomain
from domain.safety_domain import SafetyDomain
from domain.tour_domain import NavigationOutcome, TourDecision, TourDomain, TourState, TourStop
from integrations.ros2_connector import NAV2_AVAILABLE, ROS2_AVAILABLE, ROS2Connector
from services.affect_service import AffectService
from services.engagement_service import EngagementDecision, EngagementService, UtteranceClass
from services.knowledge_service import KnowledgeService
from services.memory_service import MemoryService
from services.navigation_service import NavigationService
from services.telemetry_service import TelemetryService
from runtime.model_config import ModelStackConfig

if TYPE_CHECKING:
    from agent.concierge_agent import ConciergeAgent
    from agent.tour_agent import TourAgent

logger = logging.getLogger('moretea.runtime')


@dataclass
class RobotContext:
    knowledge: KnowledgeService
    memory: MemoryService
    interaction: InteractionDomain
    safety: SafetyDomain
    navigation: NavigationService
    affect: AffectService
    telemetry: TelemetryService
    tour: TourDomain
    engagement: EngagementService


@dataclass(frozen=True)
class StartupCheck:
    name: str
    ok: bool
    detail: str
    severity: str = 'error'


@dataclass(frozen=True)
class StartupHealthReport:
    checks: tuple[StartupCheck, ...]
    navigation_ready: bool

    @property
    def ok(self) -> bool:
        return all(check.ok or check.severity == 'warning' for check in self.checks)

    def as_payload(self) -> dict[str, Any]:
        return {
            'ok': self.ok,
            'navigation_ready': self.navigation_ready,
            'checks': [
                {
                    'name': check.name,
                    'ok': check.ok,
                    'detail': check.detail,
                    'severity': check.severity,
                }
                for check in self.checks
            ],
        }

    def summary(self) -> str:
        return '; '.join(
            f"{check.name}={'ok' if check.ok else 'fail'} ({check.detail})" for check in self.checks
        )


@dataclass(frozen=True)
class ResolvedModelStack:
    stt: Any
    llm: Any
    tts: Any
    source: str
    fallback_used: bool = False
    fallback_reason: str | None = None

    def telemetry_payload(self) -> dict[str, Any]:
        payload = {
            'source': self.source,
            'fallback_used': self.fallback_used,
        }
        if self.fallback_reason:
            payload['fallback_reason'] = self.fallback_reason
        return payload


class SessionRuntime:
    REQUIRED_ENV_VARS = ('LIVEKIT_URL', 'LIVEKIT_API_KEY', 'LIVEKIT_API_SECRET')

    def __init__(
        self,
        *,
        job_ctx: JobContext | None = None,
        always_awake: bool = False,
        content_dir: Path | None = None,
        env: Mapping[str, str] | None = None,
        require_livekit_env: bool | None = None,
    ) -> None:
        self.job_ctx = job_ctx
        self.content_dir = content_dir or Path(__file__).resolve().parent.parent / 'content'
        self.env = dict(os.environ if env is None else env)
        self.require_livekit_env = (job_ctx is not None) if require_livekit_env is None else require_livekit_env
        self.model_stack_config = ModelStackConfig.from_env(self.env)
        self.telemetry = TelemetryService()
        self.knowledge = KnowledgeService(content_dir=self.content_dir)
        self.memory = MemoryService()
        self.interaction = InteractionDomain(idle_timeout_sec=5.0)
        self.safety = SafetyDomain()
        self.ros2 = ROS2Connector() if ROS2_AVAILABLE else None
        self.navigation = NavigationService(self.ros2, self.telemetry)
        self.affect = AffectService(self.ros2)
        self.tour_domain = TourDomain(self._load_stops())
        self.engagement = EngagementService(
            self.interaction,
            list(self.knowledge.persona.wake_phrases),
            self.knowledge.persona.sleep_prompt,
            wake_greeting=(
                self.knowledge.persona.greetings[0]
                if self.knowledge.persona.greetings
                else 'Hello, I am awake and ready to help.'
            ),
            always_awake=always_awake,
        )
        self.robot_context = RobotContext(
            knowledge=self.knowledge,
            memory=self.memory,
            interaction=self.interaction,
            safety=self.safety,
            navigation=self.navigation,
            affect=self.affect,
            telemetry=self.telemetry,
            tour=self.tour_domain,
            engagement=self.engagement,
        )
        self.session: AgentSession | None = None
        self.concierge_agent: ConciergeAgent | None = None
        self.tour_agent: TourAgent | None = None
        self._idle_task: asyncio.Task | None = None
        self.startup_health = self.inspect_startup_health(
            content_dir=self.content_dir,
            env=self.env,
            require_livekit_env=self.require_livekit_env,
        )

    @classmethod
    def build_test_runtime(cls, *, always_awake: bool = True) -> 'SessionRuntime':
        return cls(always_awake=always_awake, require_livekit_env=False)

    @classmethod
    def inspect_startup_health(
        cls,
        *,
        content_dir: Path | None = None,
        env: Mapping[str, str] | None = None,
        require_livekit_env: bool = False,
    ) -> StartupHealthReport:
        resolved_content_dir = content_dir or Path(__file__).resolve().parent.parent / 'content'
        checks: list[StartupCheck] = []
        current_env = dict(os.environ if env is None else env)

        if require_livekit_env:
            for key in cls.REQUIRED_ENV_VARS:
                present = bool(current_env.get(key))
                detail = 'configured' if present else 'missing'
                checks.append(StartupCheck(f'env:{key}', present, detail))

        model_stack_config = ModelStackConfig.from_env(current_env)
        if model_stack_config.use_local_models:
            checks.extend(
                [
                    StartupCheck(
                        'local_llm_base_url',
                        cls._is_valid_http_url(model_stack_config.local.llm.base_url),
                        model_stack_config.local.llm.base_url,
                    ),
                    StartupCheck(
                        'local_stt_base_url',
                        cls._is_valid_http_url(model_stack_config.local.stt.base_url),
                        model_stack_config.local.stt.base_url,
                    ),
                    StartupCheck(
                        'local_tts_base_url',
                        cls._is_valid_http_url(model_stack_config.local.tts.base_url),
                        model_stack_config.local.tts.base_url,
                    ),
                    StartupCheck(
                        'local_model_ids',
                        all(
                            [
                                bool(model_stack_config.local.llm.model),
                                bool(model_stack_config.local.stt.model),
                                bool(model_stack_config.local.tts.model),
                            ]
                        ),
                        'Local STT/LLM/TTS models configured.',
                    ),
                ]
            )

        try:
            KnowledgeService(content_dir=resolved_content_dir)
            checks.append(StartupCheck('content_files', True, 'Persona, policy, and FAQ content loaded.'))
        except Exception as exc:  # noqa: BLE001
            checks.append(StartupCheck('content_files', False, str(exc)))

        try:
            stops = cls._load_stops_from_content_dir(resolved_content_dir)
            checks.append(StartupCheck('tour_stops', True, f'{len(stops)} configured stop(s) loaded.'))
        except Exception as exc:  # noqa: BLE001
            checks.append(StartupCheck('tour_stops', False, str(exc)))

        checks.append(
            StartupCheck(
                'ros2_import',
                ROS2_AVAILABLE,
                'rclpy is available.' if ROS2_AVAILABLE else 'rclpy is not installed; stationary concierge mode only.',
                severity='warning',
            )
        )
        checks.append(
            StartupCheck(
                'nav2_import',
                NAV2_AVAILABLE,
                'nav2_simple_commander is available.' if NAV2_AVAILABLE else 'Nav2 simple commander is not installed.',
                severity='warning',
            )
        )
        return StartupHealthReport(tuple(checks), navigation_ready=ROS2_AVAILABLE and NAV2_AVAILABLE)

    @classmethod
    def _load_stops_from_content_dir(cls, content_dir: Path) -> tuple[TourStop, ...]:
        path = content_dir / 'tour_stops.yaml'
        with path.open('r', encoding='utf-8') as fh:
            raw = yaml.safe_load(fh) or {}
        if not isinstance(raw, dict):
            raise ValueError('tour_stops.yaml must contain a mapping at the top level')
        items = raw.get('stops', [])
        if not isinstance(items, list) or not items:
            raise ValueError('tour_stops.yaml must define at least one stop')

        result: list[TourStop] = []
        seen_ids: set[str] = set()
        seen_names: set[str] = set()
        for index, item in enumerate(items, start=1):
            if not isinstance(item, dict):
                raise ValueError(f'stop #{index} must be a mapping')
            for required_key in ('id', 'name', 'x', 'y', 'ow', 'narration'):
                if required_key not in item:
                    raise ValueError(f"stop #{index} is missing required field '{required_key}'")
            stop = TourStop(
                id=str(item['id']).strip(),
                name=str(item['name']).strip(),
                aliases=tuple(str(alias).strip() for alias in item.get('aliases', [])),
                x=float(item['x']),
                y=float(item['y']),
                ow=float(item['ow']),
                narration=str(item['narration']).strip(),
                faq_tags=tuple(str(tag).strip() for tag in item.get('faq_tags', [])),
                escort_only=bool(item.get('escort_only', False)),
            )
            if not stop.id:
                raise ValueError(f'stop #{index} must have a non-empty id')
            if stop.id in seen_ids:
                raise ValueError(f'duplicate stop id: {stop.id}')
            seen_ids.add(stop.id)
            if not stop.name:
                raise ValueError(f"stop '{stop.id}' must have a non-empty name")
            if not stop.narration:
                raise ValueError(f"stop '{stop.id}' must have a non-empty narration")
            for numeric_name, value in (('x', stop.x), ('y', stop.y), ('ow', stop.ow)):
                if not math.isfinite(value):
                    raise ValueError(f"stop '{stop.id}' has an invalid {numeric_name} value")
            local_names = {
                cls._normalize_location_candidate(candidate)
                for candidate in (stop.name, stop.id.replace('_', ' '), *stop.aliases)
            }
            for normalized in local_names:
                if normalized in seen_names:
                    raise ValueError(f"duplicate stop name or alias detected: {normalized}")
            seen_names.update(local_names)
            result.append(stop)
        return tuple(result)

    @staticmethod
    def _normalize_location_candidate(text: str) -> str:
        return ' '.join(text.lower().strip().split())

    @staticmethod
    def _is_valid_http_url(value: str) -> bool:
        parsed = httpx.URL(value)
        return parsed.scheme in {'http', 'https'} and bool(parsed.host)

    def _load_stops(self) -> tuple[TourStop, ...]:
        return self._load_stops_from_content_dir(self.content_dir)

    def _build_runtime_health_report(self) -> StartupHealthReport:
        checks = list(self.inspect_startup_health(
            content_dir=self.content_dir,
            env=self.env,
            require_livekit_env=self.require_livekit_env,
        ).checks)
        connector_started = bool(self.ros2 is not None and getattr(self.ros2, '_started', False))
        checks.append(
            StartupCheck(
                'ros2_connector_started',
                connector_started,
                'ROS 2 connector started.' if connector_started else 'ROS 2 connector not started.',
                severity='warning',
            )
        )
        nav2_ready = bool(self.ros2 is not None and getattr(self.ros2, '_navigator', None) is not None)
        checks.append(
            StartupCheck(
                'nav2_runtime_ready',
                nav2_ready,
                'Nav2 navigator is ready.' if nav2_ready else 'Nav2 navigator is not ready.',
                severity='warning',
            )
        )
        return StartupHealthReport(tuple(checks), navigation_ready=nav2_ready)

    def instructions_for(self, role: str) -> str:
        return self.knowledge.build_agent_instructions(
            agent_role=role,
            interaction_state=self.interaction.state,
            tour_status=self.tour_domain.status(),
            stops=self.tour_domain.stops,
            safety_state=self.safety.state,
            safety_reason=self.safety.reason,
            memory_summary=self.memory.summary(),
        )

    async def refresh_agent_instructions(self) -> None:
        if self.concierge_agent is not None:
            await self.concierge_agent.update_instructions(self.instructions_for('concierge'))
        if self.tour_agent is not None:
            await self.tour_agent.update_instructions(self.instructions_for('tour'))
        self.affect.apply_interaction_state(self.interaction.state)

    def _ensure_agents(self) -> None:
        if self.concierge_agent is None or self.tour_agent is None:
            from agent.concierge_agent import ConciergeAgent
            from agent.tour_agent import TourAgent

            self.concierge_agent = ConciergeAgent(self)
            self.tour_agent = TourAgent(self)

    def _navigation_unavailable_message(self) -> str:
        return 'Navigation is unavailable right now, so I will stay here as a stationary concierge.'

    def _movement_blocked_message(self) -> str | None:
        if self.safety.motion_allowed:
            return None
        return self.safety.reason or self._navigation_unavailable_message()

    def _build_hosted_model_stack(
        self,
        *,
        fallback_used: bool = False,
        fallback_reason: str | None = None,
    ) -> ResolvedModelStack:
        hosted = self.model_stack_config.hosted
        livekit_url = self.env.get('LIVEKIT_URL', '')
        livekit_api_key = self.env.get('LIVEKIT_API_KEY', '')
        livekit_api_secret = self.env.get('LIVEKIT_API_SECRET', '')
        return ResolvedModelStack(
            stt=inference.STT(
                model=hosted.stt_model,
                language=hosted.stt_language,
                base_url=livekit_url,
                api_key=livekit_api_key,
                api_secret=livekit_api_secret,
            ),
            llm=inference.LLM(
                model=hosted.llm_model,
                base_url=livekit_url,
                api_key=livekit_api_key,
                api_secret=livekit_api_secret,
            ),
            tts=inference.TTS(
                model=hosted.tts_model,
                voice=hosted.tts_voice,
                base_url=livekit_url,
                api_key=livekit_api_key,
                api_secret=livekit_api_secret,
            ),
            source='hosted',
            fallback_used=fallback_used,
            fallback_reason=fallback_reason,
        )

    def _build_local_model_stack(self) -> ResolvedModelStack:
        try:
            from livekit.plugins import openai
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                'Local model mode requires the OpenAI LiveKit plugin. Run uv sync after updating dependencies.'
            ) from exc

        local = self.model_stack_config.local
        llm_base_url = local.llm.base_url.rstrip('/')
        if llm_base_url.endswith('11434/v1') or llm_base_url.endswith('11434'):
            llm = openai.LLM.with_ollama(model=local.llm.model, base_url=local.llm.base_url)
        else:
            llm = openai.LLM(
                model=local.llm.model,
                api_key=local.llm.api_key,
                base_url=local.llm.base_url,
            )

        stt = openai.STT(
            model=local.stt.model,
            api_key=local.stt.api_key,
            base_url=local.stt.base_url,
            language=local.stt.language or 'en',
            use_realtime=False,
        )
        tts = openai.TTS(
            model=local.tts.model,
            voice=local.tts.voice or 'af_heart',
            api_key=local.tts.api_key,
            base_url=local.tts.base_url,
            instructions=local.tts.instructions,
            response_format=local.tts.response_format or 'pcm',
        )
        return ResolvedModelStack(stt=stt, llm=llm, tts=tts, source='local')

    async def _probe_service(self, name: str, url: str) -> StartupCheck:
        timeout = httpx.Timeout(connect=2.0, read=4.0, write=4.0, pool=2.0)
        try:
            async with httpx.AsyncClient(timeout=timeout, follow_redirects=True) as client:
                response = await client.get(url)
            ok = response.status_code < 500
            detail = f'{url} -> HTTP {response.status_code}'
            return StartupCheck(name, ok, detail, severity='warning')
        except Exception as exc:  # noqa: BLE001
            return StartupCheck(name, False, f'{url} -> {exc}', severity='warning')

    async def _check_local_model_services(self) -> tuple[StartupCheck, ...]:
        local = self.model_stack_config.local
        checks = await asyncio.gather(
            self._probe_service('local_llm_service', local.llm.health_url),
            self._probe_service('local_stt_service', local.stt.health_url),
            self._probe_service('local_tts_service', local.tts.health_url),
        )
        return tuple(checks)

    async def _resolve_model_stack(self) -> ResolvedModelStack:
        if not self.model_stack_config.use_local_models:
            return self._build_hosted_model_stack()

        health_checks = await self._check_local_model_services()
        self.telemetry.event(
            'local_model_health',
            checks=[
                {
                    'name': check.name,
                    'ok': check.ok,
                    'detail': check.detail,
                    'severity': check.severity,
                }
                for check in health_checks
            ],
        )
        failures = [check for check in health_checks if not check.ok]
        if failures:
            reason = '; '.join(check.detail for check in failures)
            if self.model_stack_config.fallback_to_hosted:
                logger.warning('Falling back to hosted models after local service probe failure: %s', reason)
                return self._build_hosted_model_stack(fallback_used=True, fallback_reason=reason)
            raise RuntimeError(f'Local model services are unavailable: {reason}')

        try:
            return self._build_local_model_stack()
        except Exception as exc:  # noqa: BLE001
            if self.model_stack_config.fallback_to_hosted:
                logger.warning('Falling back to hosted models after local model init failure: %s', exc)
                return self._build_hosted_model_stack(
                    fallback_used=True,
                    fallback_reason=str(exc),
                )
            raise

    async def start(self) -> None:
        if self.job_ctx is None:
            raise RuntimeError('JobContext is required to start the live runtime')
        self.job_ctx.log_context_fields = {'room': self.job_ctx.room.name}
        if self.ros2 is not None and not getattr(self.ros2, '_started', False):
            self.ros2.start(asyncio.get_event_loop())

        self.startup_health = self._build_runtime_health_report()
        self.telemetry.startup_health(self.startup_health)
        if not self.startup_health.ok:
            raise RuntimeError(self.startup_health.summary())
        if not self.startup_health.navigation_ready:
            self.safety.degrade(self._navigation_unavailable_message())

        self._ensure_agents()
        model_stack = await self._resolve_model_stack()
        self.telemetry.event('model_stack_selected', **model_stack.telemetry_payload())
        self.session = AgentSession(
            stt=model_stack.stt,
            llm=model_stack.llm,
            tts=model_stack.tts,
            turn_detection=MultilingualModel(),
            vad=self.job_ctx.proc.userdata['vad'],
            preemptive_generation=True,
            userdata=self.robot_context,
        )
        self._bind_session_events()
        await self.session.start(
            agent=self.concierge_agent,
            room=self.job_ctx.room,
            room_options=room_io.RoomOptions(
                audio_input=room_io.AudioInputOptions(
                    noise_cancellation=lambda params: (
                        noise_cancellation.BVCTelephony()
                        if params.participant.kind == rtc.ParticipantKind.PARTICIPANT_KIND_SIP
                        else noise_cancellation.BVC()
                    )
                )
            ),
        )
        self.affect.apply_interaction_state(self.interaction.state)
        self._idle_task = asyncio.create_task(self._idle_monitor(), name='moretea_idle_monitor')
        self.telemetry.event(
            'runtime_started',
            room=self.job_ctx.room.name,
            always_awake=self.engagement.state != self.interaction.state.SLEEPING,
            model_mode=self.model_stack_config.mode_label(),
        )

    def _bind_session_events(self) -> None:
        assert self.session is not None

        @self.session.on('user_input_transcribed')
        def _on_user_input(ev: Any) -> None:
            if getattr(ev, 'is_final', False):
                self.telemetry.event('user_input_transcribed', transcript=ev.transcript)

        @self.session.on('user_state_changed')
        def _on_user_state(ev: Any) -> None:
            self.telemetry.event('user_state_changed', old_state=ev.old_state, new_state=ev.new_state)

        @self.session.on('agent_state_changed')
        def _on_agent_state(ev: Any) -> None:
            self.telemetry.event('agent_state_changed', old_state=ev.old_state, new_state=ev.new_state)

        @self.session.on('conversation_item_added')
        def _on_item(ev: Any) -> None:
            item = getattr(ev, 'item', None)
            role = getattr(item, 'role', None)
            self.telemetry.event('conversation_item_added', role=role)

        @self.session.on('metrics_collected')
        def _on_metrics(ev: Any) -> None:
            self.telemetry.event('metrics_collected', metrics_type=type(ev.metrics).__name__)

    async def _idle_monitor(self) -> None:
        while True:
            await asyncio.sleep(0.5)
            if self.engagement.maybe_sleep():
                self.telemetry.event('idle_timeout_fired')
                await self.refresh_agent_instructions()

    def extract_user_text(self, message: Any) -> str:
        content = getattr(message, 'content', []) or []
        parts: list[str] = []
        for item in content:
            if isinstance(item, str):
                parts.append(item)
            else:
                text = getattr(item, 'text', None)
                if isinstance(text, str):
                    parts.append(text)
        return ' '.join(part.strip() for part in parts if part).strip()

    async def handle_user_turn(self, text: str) -> EngagementDecision:
        lowered = text.lower().strip()
        if any(token in lowered for token in ['stop!', 'watch out', 'danger']):
            await self.stop_robot()
            if self.session is not None:
                await self.session.say('Emergency stop acknowledged.', allow_interruptions=True)
            return EngagementDecision(classification=UtteranceClass.IGNORE, allow_response=False)

        previous_interaction_state = self.interaction.state
        captured_name = self.memory.maybe_capture_name(text)
        decision = self.engagement.handle_user_text(text)
        if captured_name:
            self.telemetry.event('memory_updated', field='user_name', value=captured_name)
            decision.instructions_changed = True
        if decision.transitioned_state is not None:
            self.telemetry.event('wake_transition', new_state=decision.transitioned_state.value)
            decision.instructions_changed = True
        if decision.allow_response:
            if self.tour_domain.state == TourState.NAVIGATING:
                self.engagement.mark_guiding()
            else:
                self.engagement.mark_conversing()
        else:
            self.telemetry.event('ignored_turn', classification=decision.classification.value)
        if self.interaction.state != previous_interaction_state:
            decision.transitioned_state = self.interaction.state
            decision.instructions_changed = True
        return decision

    async def _execute_navigation_decision(self, decision: TourDecision) -> str:
        if decision.target_stop is None:
            return decision.speech
        self.telemetry.event('navigation_started', stop_id=decision.target_stop.id, stop_name=decision.target_stop.name)
        self.engagement.mark_guiding()
        self.affect.apply_interaction_state(self.interaction.state)
        result = await self.navigation.navigate_to_stop(decision.target_stop)
        self.affect.apply_navigation_outcome(result.outcome)
        if result.event_note:
            self.telemetry.event('navigation_event_note', note=result.event_note)
            if self.session is not None:
                await self.session.say(result.event_note, allow_interruptions=True)
        follow_up = self.tour_domain.handle_navigation_result(result)
        if result.outcome == NavigationOutcome.UNAVAILABLE:
            self.safety.degrade(self._navigation_unavailable_message())
        if follow_up.target_stop is not None and follow_up.target_stop.id != decision.target_stop.id:
            nested = await self._execute_navigation_decision(follow_up)
            return f'{follow_up.speech} {nested}'.strip()
        return follow_up.speech

    async def start_tour(self) -> str:
        self.memory.remember_preference('interaction_mode', 'tour')
        blocked = self._movement_blocked_message()
        await self.refresh_agent_instructions()
        if blocked is not None:
            return blocked
        decision = self.tour_domain.start_tour()
        await self.refresh_agent_instructions()
        return await self._execute_navigation_decision(decision)

    async def go_to_location(self, query: str) -> str:
        self.memory.remember_preference('interaction_mode', 'tour')
        blocked = self._movement_blocked_message()
        if blocked is not None:
            await self.refresh_agent_instructions()
            return blocked
        decision = self.tour_domain.go_to_location(query)
        await self.refresh_agent_instructions()
        return await self._execute_navigation_decision(decision)

    async def pause_tour(self) -> str:
        await self.navigation.cancel()
        decision = self.tour_domain.pause()
        self.engagement.mark_conversing()
        await self.refresh_agent_instructions()
        return decision.speech

    async def resume_tour(self) -> str:
        self.memory.remember_preference('interaction_mode', 'tour')
        blocked = self._movement_blocked_message()
        if blocked is not None:
            await self.refresh_agent_instructions()
            return blocked
        decision = self.tour_domain.resume()
        await self.refresh_agent_instructions()
        return await self._execute_navigation_decision(decision)

    async def stop_robot(self) -> str:
        await self.navigation.cancel()
        decision = self.tour_domain.emergency_stop()
        self.telemetry.event('emergency_stop_triggered')
        self.engagement.mark_blocked()
        await self.refresh_agent_instructions()
        return decision.speech

    def navigation_status(self) -> str:
        blocked = self._movement_blocked_message()
        if blocked is not None and not self.navigation.is_navigating:
            return blocked
        return self.navigation.status_text()

    async def lookup_garage_faq(self, question: str) -> str:
        self.memory.remember_preference('interaction_mode', 'faq')
        await self.refresh_agent_instructions()
        answer = self.knowledge.lookup_faq(question)
        self.telemetry.event('faq_lookup', question=question, found=bool(answer))
        if answer is None:
            return 'I do not have a curated FAQ answer for that yet, but I can still try to help using my Garage knowledge.'
        return answer

    def get_tour_status(self) -> str:
        status = self.tour_domain.status()
        lines = [f'Tour state: {status.state.value}.']
        if status.current_stop_id:
            lines.append(f'Current stop: {status.current_stop_id}.')
        if status.next_stop_id:
            lines.append(f'Next stop: {status.next_stop_id}.')
        if status.last_completed_stop_id:
            lines.append(f'Last completed stop: {status.last_completed_stop_id}.')
        if status.skipped_stop_ids:
            lines.append('Skipped stops: ' + ', '.join(status.skipped_stop_ids) + '.')
        if status.last_navigation_outcome is not None:
            lines.append(f'Last navigation outcome: {status.last_navigation_outcome.value}.')
        return ' '.join(lines)
