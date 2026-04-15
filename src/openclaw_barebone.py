from __future__ import annotations

import asyncio
import logging
import math
import os
import re
import struct
import time

import httpx
from collections import deque
from collections.abc import AsyncIterable
from contextlib import suppress
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

import httpx

try:
    from dotenv import load_dotenv
except ModuleNotFoundError:  # pragma: no cover - optional dependency
    load_dotenv = None
try:
    import sounddevice

    SOUNDDEVICE_AVAILABLE = True
except ModuleNotFoundError:  # pragma: no cover - optional dependency
    sounddevice = None  # type: ignore[assignment]
    SOUNDDEVICE_AVAILABLE = False

from livekit.agents import (
    Agent,
    AgentServer,
    AgentSession,
    JobContext,
    JobProcess,
    cli,
    inference,
    llm,
)
from livekit.plugins import openai, silero
from livekit.plugins.turn_detector.multilingual import MultilingualModel

from runtime.model_config import ModelStackConfig

logger = logging.getLogger(__name__)


class AttentionState(str, Enum):
    PASSIVE = 'passive'
    ENGAGED = 'engaged'
    COOLDOWN = 'cooldown'


@dataclass(frozen=True)
class AttentionDecision:
    allow_response: bool
    normalized_text: str = ''
    prompt_to_say: str | None = None
    system_note: str | None = None
    instructions_changed: bool = False
    used_ambient_context: bool = False
    classification: str = 'ignore'
    state: AttentionState = AttentionState.PASSIVE


@dataclass(frozen=True)
class AmbientUtterance:
    text: str
    created_at: float


class ThinkingCue:
    async def play_start(self) -> None:
        raise NotImplementedError


class NullThinkingCue(ThinkingCue):
    async def play_start(self) -> None:
        return


class LocalThinkingCue(ThinkingCue):
    _SAMPLE_RATE = 48_000
    _ATTACK_SEC = 0.01
    _RELEASE_SEC = 0.04
    _DURATION_SEC = 0.18
    _AMPLITUDE = 14_000
    _FREQ_SEGMENTS = (
        (0.09, 880.0),
        (0.09, 1320.0),
    )

    def __init__(self, *, enabled: bool, volume: float) -> None:
        self._enabled = enabled
        self._volume = max(0.0, min(volume, 1.0))
        self._cue_samples = self._build_cue_samples()
        self._play_task: asyncio.Task[None] | None = None

    async def play_start(self) -> None:
        if not self._enabled:
            return
        if not SOUNDDEVICE_AVAILABLE:
            logger.warning('Thinking cue unavailable: sounddevice is not installed in this runtime.')
            return
        if self._play_task is not None and not self._play_task.done():
            return
        loop = asyncio.get_running_loop()
        self._play_task = loop.create_task(self._play_once())
        self._play_task.add_done_callback(self._log_playback_failure)

    async def _play_once(self) -> None:
        await asyncio.to_thread(self._play_blocking)

    def _play_blocking(self) -> None:
        if sounddevice is None:  # pragma: no cover - guarded by SOUNDDEVICE_AVAILABLE
            return
        pcm = struct.unpack(f'<{len(self._cue_samples) // 2}h', self._cue_samples)
        normalized = [(sample / 32768.0) * self._volume for sample in pcm]
        sounddevice.play(normalized, samplerate=self._SAMPLE_RATE, blocking=False)

    @staticmethod
    def _log_playback_failure(task: asyncio.Task[None]) -> None:
        try:
            task.result()
        except Exception:  # pragma: no cover - playback backend errors are environment-specific
            logger.exception('Failed to play local thinking cue.')

    @classmethod
    def _build_cue_samples(cls) -> bytes:
        total_samples = int(cls._SAMPLE_RATE * cls._DURATION_SEC)
        segment_edges: list[tuple[int, float]] = []
        consumed = 0
        for duration_sec, frequency_hz in cls._FREQ_SEGMENTS:
            segment_samples = int(cls._SAMPLE_RATE * duration_sec)
            consumed += segment_samples
            segment_edges.append((consumed, frequency_hz))
        if segment_edges:
            segment_edges[-1] = (total_samples, segment_edges[-1][1])

        samples: list[int] = []
        attack_samples = max(1, int(cls._SAMPLE_RATE * cls._ATTACK_SEC))
        release_samples = max(1, int(cls._SAMPLE_RATE * cls._RELEASE_SEC))

        for index in range(total_samples):
            frequency_hz = segment_edges[-1][1]
            for edge_sample, candidate_hz in segment_edges:
                if index < edge_sample:
                    frequency_hz = candidate_hz
                    break
            envelope = 1.0
            if index < attack_samples:
                envelope = index / attack_samples
            elif index >= total_samples - release_samples:
                envelope = max(0.0, (total_samples - index) / release_samples)
            sample = int(
                cls._AMPLITUDE
                * envelope
                * math.sin(2.0 * math.pi * frequency_hz * (index / cls._SAMPLE_RATE))
            )
            samples.append(sample)
        return struct.pack(f'<{len(samples)}h', *samples)


class BareboneAttentionController:
    _EMERGENCY_PHRASES = ('stop', 'stop!', 'watch out', 'danger', 'emergency')
    _DIRECT_PATTERNS = (
        'hey',
        'hi',
        'hello',
        'can you',
        'could you',
        'would you',
        'will you',
        'help me',
        'please',
        'tell me',
        'show me',
        'guide me',
        'take me',
        'bring me',
        'follow me',
        'come here',
        'listen to me',
        'look at me',
        'look here',
        'i need',
        'i want',
        'where is',
        'where are',
        'what is',
        'what are',
        'who is',
        'how do',
        'how can',
        'why is',
        'when is',
        'are you',
        'do you',
        'could i',
        'can i',
        'thanks moretea',
    )
    _QUESTION_STARTERS = (
        'what',
        'where',
        'when',
        'why',
        'who',
        'how',
        'can',
        'could',
        'would',
        'will',
        'do',
        'are',
        'is',
        'should',
    )
    _IMPERATIVE_STARTERS = (
        'help',
        'tell',
        'show',
        'guide',
        'take',
        'bring',
        'follow',
        'come',
        'stop',
        'wait',
        'explain',
        'find',
        'answer',
        'look',
        'listen',
    )
    _WEAK_PHRASES = {'hmm', 'uh', 'um'}

    def __init__(
        self,
        keywords: list[str],
        sleep_prompt: str,
        *,
        idle_timeout_sec: float,
        passive_context_window_sec: float,
        cooldown_sec: float,
        allow_direct_cold_start: bool,
        low_key_cues: bool,
        time_fn: Any = time.monotonic,
    ) -> None:
        self._keywords = [self._normalize(item) for item in keywords if self._normalize(item)]
        self._sleep_prompt = sleep_prompt.strip()
        self._idle_timeout_sec = max(0.0, idle_timeout_sec)
        self._passive_context_window_sec = max(0.0, passive_context_window_sec)
        self._cooldown_sec = max(0.0, cooldown_sec)
        self._allow_direct_cold_start = allow_direct_cold_start
        self._low_key_cues = low_key_cues
        self._time_fn = time_fn
        self._state = AttentionState.PASSIVE
        self._last_directed_at: float | None = None
        self._ambient_buffer: deque[AmbientUtterance] = deque()
        self._pending_context: str | None = None

    @property
    def state(self) -> AttentionState:
        return self._state

    def recent_ambient_texts(self) -> tuple[str, ...]:
        return tuple(item.text for item in self._ambient_buffer)

    @staticmethod
    def _normalize(text: str) -> str:
        text = text.lower().strip()
        text = re.sub(r'[^a-z0-9\s]', ' ', text)
        return re.sub(r'\s+', ' ', text).strip()

    def _contains_keyword(self, normalized_text: str) -> bool:
        words = normalized_text.split()
        text_with_boundaries = f" {' '.join(words)} "
        for keyword in self._keywords:
            keyword_words = keyword.split()
            if not keyword_words:
                continue
            if f" {' '.join(keyword_words)} " in text_with_boundaries:
                return True
        return False

    def _contains_emergency_phrase(self, normalized_text: str) -> bool:
        return any(phrase in normalized_text for phrase in self._EMERGENCY_PHRASES)

    def _refresh_state(self, now: float) -> bool:
        previous = self._state
        if self._last_directed_at is None:
            self._state = AttentionState.PASSIVE
        else:
            elapsed = now - self._last_directed_at
            if elapsed >= self._idle_timeout_sec + self._cooldown_sec:
                self._state = AttentionState.PASSIVE
            elif elapsed >= self._idle_timeout_sec:
                self._state = AttentionState.COOLDOWN
            else:
                self._state = AttentionState.ENGAGED
        if self._state == AttentionState.PASSIVE:
            self._pending_context = None
        return self._state != previous

    def _mark_directed(self, now: float) -> None:
        self._last_directed_at = now
        self._state = AttentionState.ENGAGED

    def _prune_ambient_buffer(self, now: float) -> None:
        cutoff = now - self._passive_context_window_sec
        while self._ambient_buffer and self._ambient_buffer[0].created_at < cutoff:
            self._ambient_buffer.popleft()

    def _remember_ambient(self, normalized_text: str, now: float) -> None:
        if not normalized_text:
            return
        self._ambient_buffer.append(AmbientUtterance(text=normalized_text, created_at=now))
        self._prune_ambient_buffer(now)

    def _ambient_snapshot(self) -> str | None:
        if not self._ambient_buffer:
            return None
        recent = [item.text for item in list(self._ambient_buffer)[-4:]]
        return '; '.join(recent)

    def _question_or_request_score(self, normalized_text: str) -> int:
        score = 0
        words = normalized_text.split()
        if normalized_text in self._WEAK_PHRASES:
            return 0
        if any(pattern in normalized_text for pattern in self._DIRECT_PATTERNS):
            score += 4
        if any(normalized_text.startswith(f'{starter} ') for starter in self._QUESTION_STARTERS):
            score += 3
        if normalized_text.endswith('?'):
            score += 2
        if re.search(r'\b(you|your|yours)\b', normalized_text):
            score += 2
        if words and words[0] in self._IMPERATIVE_STARTERS and len(words) <= 8:
            score += 3
        if len(words) <= 8 and score > 0:
            score += 1
        if normalized_text.startswith('hey ') and len(words) <= 2:
            score = max(0, score - 2)
        if any(marker in normalized_text for marker in ('he said', 'she said', 'they said', 'we were', 'lets ')):
            score = max(0, score - 1)
        return score

    def _classify(self, normalized_text: str, *, explicit_wake: bool) -> str:
        if self._contains_emergency_phrase(normalized_text):
            return 'emergency'
        if explicit_wake:
            return 'wake'
        score = self._question_or_request_score(normalized_text)
        if score >= 6:
            return 'direct'
        if score >= 3:
            return 'unclear'
        return 'ambient'

    def _maybe_capture_passive_context(self) -> bool:
        snapshot = self._ambient_snapshot()
        self._ambient_buffer.clear()
        if not snapshot:
            self._pending_context = None
            return False
        self._pending_context = snapshot
        return True

    def build_runtime_instructions(self, base_instructions: str) -> str:
        state_guidance = {
            AttentionState.PASSIVE: (
                'Attention mode: passive. Only answer turns that were classified as clearly directed to you.'
            ),
            AttentionState.ENGAGED: (
                'Attention mode: engaged. Treat short follow-up turns as part of the same active conversation.'
            ),
            AttentionState.COOLDOWN: (
                'Attention mode: cooldown. Prefer brief, natural replies if the next turn sounds like a follow-up.'
            ),
        }[self._state]
        parts = [base_instructions.strip(), state_guidance]
        if self._pending_context:
            parts.append(
                'Recent nearby context before the user directly engaged you: '
                f'{self._pending_context}. Use it only if clearly relevant. '
                'Do not mention passive listening or claim durable memory unless asked.'
            )
        return '\n\n'.join(part for part in parts if part)

    def evaluate(self, text: str) -> AttentionDecision:
        now = self._time_fn()
        self._prune_ambient_buffer(now)
        state_changed = self._refresh_state(now)
        normalized = self._normalize(text)
        if not normalized:
            return AttentionDecision(
                allow_response=False,
                normalized_text=normalized,
                instructions_changed=state_changed,
                classification='noise',
                state=self._state,
            )

        explicit_wake = self._contains_keyword(normalized)
        classification = self._classify(normalized, explicit_wake=explicit_wake)

        if classification == 'emergency':
            self._mark_directed(now)
            return AttentionDecision(
                allow_response=True,
                normalized_text=normalized,
                system_note='Treat this as urgent safety-related speech. Respond immediately and prioritize safety.',
                instructions_changed=True,
                classification=classification,
                state=self._state,
            )

        if explicit_wake:
            used_context = self._maybe_capture_passive_context() if self._state == AttentionState.PASSIVE else False
            self._mark_directed(now)
            return AttentionDecision(
                allow_response=True,
                normalized_text=normalized,
                instructions_changed=True,
                used_ambient_context=used_context,
                classification=classification,
                state=self._state,
            )

        if self._state == AttentionState.ENGAGED:
            self._mark_directed(now)
            return AttentionDecision(
                allow_response=True,
                normalized_text=normalized,
                instructions_changed=state_changed,
                classification='follow_up',
                state=self._state,
            )

        if self._state == AttentionState.COOLDOWN and classification in {'direct', 'unclear'}:
            self._mark_directed(now)
            return AttentionDecision(
                allow_response=True,
                normalized_text=normalized,
                instructions_changed=True,
                classification='follow_up',
                state=self._state,
            )

        if self._state == AttentionState.PASSIVE and self._allow_direct_cold_start and classification == 'direct':
            used_context = self._maybe_capture_passive_context()
            self._mark_directed(now)
            return AttentionDecision(
                allow_response=True,
                normalized_text=normalized,
                instructions_changed=True,
                used_ambient_context=used_context,
                classification=classification,
                state=self._state,
            )

        self._remember_ambient(normalized, now)
        prompt_to_say: str | None = None
        if self._low_key_cues and classification == 'unclear' and self._sleep_prompt and not self._allow_direct_cold_start:
            prompt_to_say = self._sleep_prompt
        return AttentionDecision(
            allow_response=False,
            normalized_text=normalized,
            prompt_to_say=prompt_to_say,
            instructions_changed=state_changed,
            classification=classification,
            state=self._state,
        )


def _load_env() -> None:
    if load_dotenv is None:
        return

    here = Path(__file__).resolve()
    candidates = [
        Path.cwd() / '.env.local',
        here.parents[1] / '.env.local',
    ]
    seen: set[Path] = set()
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved in seen or not resolved.exists():
            continue
        seen.add(resolved)
        load_dotenv(resolved, override=False)


_load_env()
server = AgentServer()


class BareboneMoreTeaAgent(Agent):
    def __init__(
        self,
        *,
        attention: BareboneAttentionController | None = None,
        thinking_cue: ThinkingCue | None = None,
    ) -> None:
        self._base_instructions = os.getenv(
            'MORETEA_BAREBONE_INSTRUCTIONS',
            (
                'Keep replies concise and conversational. '
                'OpenClaw owns persona, memory, and behavior. '
                'Do not invent hidden tools or internal system details unless the user asks directly.'
            ),
        )
        attention_mode = _env('MORETEA_ATTENTION_MODE', 'session').lower()
        self._thinking_cue = thinking_cue or NullThinkingCue()
        self._attention = attention or BareboneAttentionController(
            _wake_keywords(),
            _env_allow_empty('MORETEA_SLEEP_PROMPT', "Say 'hey', 'hi', 'hello', or 'Hey MoreTea' to wake me up."),
            idle_timeout_sec=_env_float('MORETEA_IDLE_TIMEOUT_SEC', 20.0),
            passive_context_window_sec=_env_float('MORETEA_PASSIVE_CONTEXT_WINDOW_SEC', 20.0),
            cooldown_sec=_env_float('MORETEA_SLEEP_PROMPT_COOLDOWN_SEC', 4.0),
            allow_direct_cold_start=_env_bool('MORETEA_ALLOW_DIRECT_COLD_START', True),
            low_key_cues=_env_bool('MORETEA_LOW_KEY_CUES', True),
        )
        if attention_mode and attention_mode != 'session':
            self._base_instructions += ' Attention mode override requested, but barebone currently only implements session attention.'
        super().__init__(
            instructions=self._attention.build_runtime_instructions(self._base_instructions),
            id=os.getenv('MORETEA_BAREBONE_AGENT_ID', 'moretea_barebone_agent'),
        )

    @staticmethod
    def extract_user_text(message: Any) -> str:
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

    async def on_user_turn_completed(self, turn_ctx: llm.ChatContext, new_message: llm.ChatMessage) -> None:
        decision = self._attention.evaluate(self.extract_user_text(new_message))
        if decision.instructions_changed:
            updated = self._attention.build_runtime_instructions(self._base_instructions)
            if updated != self.instructions:
                await self.update_instructions(updated)
        if decision.system_note and turn_ctx is not None:
            turn_ctx.add_message(role='system', content=decision.system_note)
        if decision.allow_response:
            try:
                await self._thinking_cue.play_start()
            except Exception:  # pragma: no cover - defensive guard for custom cue providers
                logger.exception('Thinking cue playback failed before agent response.')
            return
        if decision.prompt_to_say:
            await self.session.say(decision.prompt_to_say, allow_interruptions=True)
        raise llm.StopResponse()

    async def llm_node(
        self,
        chat_ctx: llm.ChatContext,
        tools: list[llm.FunctionTool],
        model_settings: Any = None,
    ) -> AsyncIterable[llm.ChatChunk]:
        phrase = _env('MORETEA_SLOW_RESPONSE_CUE', '')
        delay = _env_float('MORETEA_SLOW_RESPONSE_DELAY_SEC', 7.0)

        if not phrase:
            async for chunk in super().llm_node(chat_ctx, tools, model_settings):
                yield chunk
            return

        first_token = asyncio.Event()

        async def _say_if_slow() -> None:
            await asyncio.sleep(delay)
            if not first_token.is_set():
                with suppress(Exception):
                    await self.session.say(phrase, allow_interruptions=True)

        cue_task = asyncio.create_task(_say_if_slow())
        try:
            async for chunk in super().llm_node(chat_ctx, tools, model_settings):
                first_token.set()
                yield chunk
        finally:
            cue_task.cancel()
            with suppress(asyncio.CancelledError):
                await cue_task


def _env(name: str, default: str = '') -> str:
    value = os.getenv(name)
    if value is None:
        return default
    stripped = value.strip()
    return stripped or default


def _env_allow_empty(name: str, default: str = '') -> str:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip()


def _env_bool(name: str, default: bool = False) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _env_float(name: str, default: float) -> float:
    value = _env(name)
    if not value:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def _env_int(name: str, default: int) -> int:
    value = _env(name)
    if not value:
        return default
    try:
        return int(value)
    except ValueError:
        return default


def _wake_keywords() -> list[str]:
    raw = _env('MORETEA_WAKE_KEYWORDS')
    default_keywords = [
        'hey',
        'hi',
        'hello',
        'moretea',
        'more tea',
        'hey moretea',
        'hey more tea',
        'morti',
        'morty',
    ]
    if not raw:
        return default_keywords
    keywords = [item.strip() for item in raw.split(',') if item.strip()]
    return keywords or default_keywords


def _session_kwargs(ctx: JobContext) -> dict[str, object]:
    return {
        'stt': _stt(),
        'llm': _openclaw_llm(),
        'tts': _tts(),
        'turn_detection': MultilingualModel(),
        'vad': ctx.proc.userdata['vad'],
        'preemptive_generation': False,
        'allow_interruptions': _env_bool('MORETEA_ALLOW_INTERRUPTIONS', True),
        'min_interruption_duration': _env_float('MORETEA_MIN_INTERRUPTION_DURATION_SEC', 0.12),
        'min_interruption_words': _env_int('MORETEA_MIN_INTERRUPTION_WORDS', 0),
    }


def _require(name: str) -> str:
    value = _env(name)
    if not value:
        raise RuntimeError(f'Missing required environment variable: {name}')
    return value


def _openclaw_llm() -> openai.LLM:
    base_url = _env('MORETEA_OPENCLAW_URL') or _env('HOSTED_LLM_BASE_URL')
    model = _env('MORETEA_OPENCLAW_MODEL') or _env('HOSTED_LLM_MODEL', 'openclaw')
    api_key = _env('MORETEA_OPENCLAW_TOKEN') or _env('HOSTED_LLM_API_KEY', 'openclaw')
    agent_id = _env('MORETEA_OPENCLAW_AGENT_ID') or _env('HOSTED_LLM_AGENT_ID')

    if not base_url:
        raise RuntimeError(
            'Missing OpenClaw base URL. Set MORETEA_OPENCLAW_URL or HOSTED_LLM_BASE_URL.'
        )

    kwargs: dict[str, object] = {
        'model': model,
        'api_key': api_key,
        'base_url': base_url,
        'timeout': httpx.Timeout(
            connect=15.0,
            read=_env_float('MORETEA_OPENCLAW_READ_TIMEOUT_SEC', 180.0),
            write=10.0,
            pool=5.0,
        ),
    }
    if agent_id:
        kwargs['extra_headers'] = {'x-openclaw-agent-id': agent_id}
    return openai.LLM(**kwargs)


def _model_stack_config() -> ModelStackConfig:
    return ModelStackConfig.from_env(os.environ)


def _stt() -> object:
    config = _model_stack_config()
    if config.use_local_models:
        return openai.STT(
            model=config.local.stt.model,
            api_key=config.local.stt.api_key,
            base_url=config.local.stt.base_url,
            language=config.local.stt.language or 'en',
            use_realtime=False,
        )

    _require('LIVEKIT_API_KEY')
    _require('LIVEKIT_API_SECRET')
    kwargs: dict[str, object] = {
        'model': config.hosted.stt_model,
        'language': config.hosted.stt_language,
    }
    inference_url = _env('LIVEKIT_INFERENCE_URL')
    if inference_url:
        kwargs['base_url'] = inference_url
    inference_api_key = _env('LIVEKIT_INFERENCE_API_KEY')
    if inference_api_key:
        kwargs['api_key'] = inference_api_key
    inference_api_secret = _env('LIVEKIT_INFERENCE_API_SECRET')
    if inference_api_secret:
        kwargs['api_secret'] = inference_api_secret
    return inference.STT(**kwargs)


def _tts() -> object:
    config = _model_stack_config()
    if config.use_local_models:
        return openai.TTS(
            model=config.local.tts.model,
            api_key=config.local.tts.api_key,
            base_url=config.local.tts.base_url,
            voice=config.local.tts.voice or 'af_heart',
            instructions=config.local.tts.instructions,
            response_format=config.local.tts.response_format or 'pcm',
        )

    _require('LIVEKIT_API_KEY')
    _require('LIVEKIT_API_SECRET')
    kwargs: dict[str, object] = {
        'model': config.hosted.tts_model,
        'voice': config.hosted.tts_voice,
    }
    inference_url = _env('LIVEKIT_INFERENCE_URL')
    if inference_url:
        kwargs['base_url'] = inference_url
    inference_api_key = _env('LIVEKIT_INFERENCE_API_KEY')
    if inference_api_key:
        kwargs['api_key'] = inference_api_key
    inference_api_secret = _env('LIVEKIT_INFERENCE_API_SECRET')
    if inference_api_secret:
        kwargs['api_secret'] = inference_api_secret
    return inference.TTS(**kwargs)


def prewarm(proc: JobProcess) -> None:
    proc.userdata['vad'] = silero.VAD.load()


server.setup_fnc = prewarm


@server.rtc_session(agent_name=os.getenv('MORETEA_BAREBONE_AGENT_NAME', 'moretea-barebone'))
async def barebone_agent(ctx: JobContext) -> None:
    await ctx.connect()
    session = AgentSession(**_session_kwargs(ctx))
    thinking_cue = LocalThinkingCue(
        enabled=_env_bool('MORETEA_THINKING_CUE_ENABLED', True),
        volume=_env_float('MORETEA_THINKING_CUE_VOLUME', 0.35),
    )
    await session.start(agent=BareboneMoreTeaAgent(thinking_cue=thinking_cue), room=ctx.room)


if __name__ == '__main__':
    cli.run_app(server)
