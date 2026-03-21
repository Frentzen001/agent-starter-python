from __future__ import annotations

from pathlib import Path

import pytest

from runtime.model_config import ModelStackConfig
from runtime.session_runtime import SessionRuntime, StartupCheck


PERSONA_YAML = """name: MoreTea
role: Pet Robot Assistant
voice_style: friendly, concise
wake_phrases:
  - moretea
greeting:
  - Hello
sleep_prompt: Say 'Hey MoreTea' to wake me up.
tour_completion_prompt: Tour complete.
system_principles:
  - Be helpful.
"""

POLICIES_YAML = """rules:
  - Always be safe.
"""

FAQ_YAML = """faqs:
  - question: Where do I start?
    answer: Start with a committee member.
    keywords: [start]
"""

STOPS_YAML = """stops:
  - id: entrance
    name: Entrance
    aliases: [front door]
    x: 0.0
    y: 0.0
    ow: 1.0
    narration: Start here.
"""


def _write_content(tmp_path: Path) -> None:
    (tmp_path / 'robot_persona.yaml').write_text(PERSONA_YAML, encoding='utf-8')
    (tmp_path / 'garage_policies.yaml').write_text(POLICIES_YAML, encoding='utf-8')
    (tmp_path / 'garage_faq.yaml').write_text(FAQ_YAML, encoding='utf-8')
    (tmp_path / 'tour_stops.yaml').write_text(STOPS_YAML, encoding='utf-8')


def test_model_stack_config_uses_local_defaults_when_enabled() -> None:
    config = ModelStackConfig.from_env({'USE_LOCAL_MODELS': '1'})

    assert config.use_local_models is True
    assert config.fallback_to_hosted is True
    assert config.local.llm.base_url == 'http://127.0.0.1:11434/v1'
    assert config.local.llm.health_url == 'http://127.0.0.1:11434/api/tags'
    assert config.local.stt.health_url == 'http://127.0.0.1:8000/models'
    assert config.local.tts.health_url == 'http://127.0.0.1:8880/models'


def test_startup_health_includes_local_model_checks_when_enabled(tmp_path: Path) -> None:
    _write_content(tmp_path)
    report = SessionRuntime.inspect_startup_health(
        content_dir=tmp_path,
        env={
            'USE_LOCAL_MODELS': '1',
            'LIVEKIT_URL': 'wss://example.test',
            'LIVEKIT_API_KEY': 'key',
            'LIVEKIT_API_SECRET': 'secret',
        },
        require_livekit_env=True,
    )

    assert any(check.name == 'local_llm_base_url' and check.ok for check in report.checks)
    assert any(check.name == 'local_stt_base_url' and check.ok for check in report.checks)
    assert any(check.name == 'local_tts_base_url' and check.ok for check in report.checks)


@pytest.mark.asyncio
async def test_resolve_model_stack_falls_back_to_hosted_when_local_services_fail(monkeypatch) -> None:
    runtime = SessionRuntime(
        always_awake=True,
        require_livekit_env=False,
        env={
            'USE_LOCAL_MODELS': '1',
            'LOCAL_MODEL_FALLBACK_TO_LIVEKIT': '1',
            'LIVEKIT_URL': 'wss://example.test',
            'LIVEKIT_API_KEY': 'key',
            'LIVEKIT_API_SECRET': 'secret',
        },
    )

    async def fake_checks() -> tuple[StartupCheck, ...]:
        return (StartupCheck('local_llm_service', False, 'llm down', severity='warning'),)

    monkeypatch.setattr(runtime, '_check_local_model_services', fake_checks)

    stack = await runtime._resolve_model_stack()

    assert stack.source == 'hosted'
    assert stack.fallback_used is True
    assert stack.fallback_reason == 'llm down'


@pytest.mark.asyncio
async def test_resolve_model_stack_raises_when_fallback_disabled(monkeypatch) -> None:
    runtime = SessionRuntime(
        always_awake=True,
        require_livekit_env=False,
        env={
            'USE_LOCAL_MODELS': '1',
            'LOCAL_MODEL_FALLBACK_TO_LIVEKIT': '0',
        },
    )

    async def fake_checks() -> tuple[StartupCheck, ...]:
        return (StartupCheck('local_tts_service', False, 'tts down', severity='warning'),)

    monkeypatch.setattr(runtime, '_check_local_model_services', fake_checks)

    with pytest.raises(RuntimeError, match='tts down'):
        await runtime._resolve_model_stack()
