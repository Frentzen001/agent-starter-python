from __future__ import annotations

from pathlib import Path

import pytest

from domain.interaction_domain import InteractionState
from runtime.session_runtime import SessionRuntime


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
  - Do not pretend to move.
"""

FAQ_YAML = """faqs:
  - question: Where do I start?
    answer: Start with a committee member.
    keywords: [start]
"""


VALID_STOPS_YAML = """stops:
  - id: entrance
    name: Entrance
    aliases: [front door]
    x: 0.0
    y: 0.0
    ow: 1.0
    narration: Start here.
"""


DUPLICATE_STOPS_YAML = """stops:
  - id: entrance
    name: Entrance
    aliases: [front door]
    x: 0.0
    y: 0.0
    ow: 1.0
    narration: Start here.
  - id: entrance
    name: Another Entrance
    aliases: [second door]
    x: 1.0
    y: 1.0
    ow: 1.0
    narration: Duplicate.
"""


def _write_content(tmp_path: Path, *, stops_yaml: str = VALID_STOPS_YAML) -> None:
    (tmp_path / 'robot_persona.yaml').write_text(PERSONA_YAML, encoding='utf-8')
    (tmp_path / 'garage_policies.yaml').write_text(POLICIES_YAML, encoding='utf-8')
    (tmp_path / 'garage_faq.yaml').write_text(FAQ_YAML, encoding='utf-8')
    (tmp_path / 'tour_stops.yaml').write_text(stops_yaml, encoding='utf-8')


def test_startup_health_passes_with_valid_content_and_env(tmp_path: Path) -> None:
    _write_content(tmp_path)
    report = SessionRuntime.inspect_startup_health(
        content_dir=tmp_path,
        env={
            'LIVEKIT_URL': 'wss://example.test',
            'LIVEKIT_API_KEY': 'key',
            'LIVEKIT_API_SECRET': 'secret',
        },
        require_livekit_env=True,
    )
    assert report.ok is True
    assert any(check.name == 'content_files' and check.ok for check in report.checks)
    assert any(check.name == 'tour_stops' and check.ok for check in report.checks)


def test_startup_health_fails_for_duplicate_stop_ids(tmp_path: Path) -> None:
    _write_content(tmp_path, stops_yaml=DUPLICATE_STOPS_YAML)
    report = SessionRuntime.inspect_startup_health(content_dir=tmp_path, require_livekit_env=False)
    assert report.ok is False
    assert any(check.name == 'tour_stops' and not check.ok for check in report.checks)


@pytest.mark.asyncio
async def test_always_awake_runtime_bypasses_sleep_gating() -> None:
    runtime = SessionRuntime.build_test_runtime(always_awake=True)
    decision = await runtime.handle_user_turn('hello there')
    assert decision.allow_response is True
    assert runtime.interaction.state == InteractionState.CONVERSING


def test_memory_summary_appears_only_after_capture() -> None:
    runtime = SessionRuntime.build_test_runtime(always_awake=True)
    before = runtime.instructions_for('concierge')
    assert '## Session Memory' not in before

    runtime.memory.maybe_capture_name('My name is Alex')
    runtime.memory.remember_preference('interaction_mode', 'tour')

    after = runtime.instructions_for('concierge')
    assert '## Session Memory' in after
    assert 'Known user name: Alex.' in after
    assert 'tour or navigation help' in after
