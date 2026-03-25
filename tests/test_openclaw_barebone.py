import asyncio
from types import SimpleNamespace

import pytest
from livekit.agents import llm

from src.openclaw_barebone import AttentionState, BareboneAttentionController, BareboneMoreTeaAgent


class Clock:
    def __init__(self, now: float = 100.0) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


class FakeSession:
    def __init__(self) -> None:
        self.spoken: list[tuple[str, bool]] = []

    async def say(self, text: str, *, allow_interruptions: bool) -> None:
        self.spoken.append((text, allow_interruptions))


class BareboneAgentHarness(BareboneMoreTeaAgent):
    def __init__(self, *, attention: BareboneAttentionController, session: FakeSession) -> None:
        self._test_session = session
        super().__init__(attention=attention)

    @property
    def session(self) -> FakeSession:
        return self._test_session


def build_attention(*, clock: Clock | None = None, allow_direct_cold_start: bool = True) -> BareboneAttentionController:
    return BareboneAttentionController(
        ['moretea', 'more tea', 'hey moretea', 'hey more tea'],
        '',
        idle_timeout_sec=20.0,
        passive_context_window_sec=20.0,
        cooldown_sec=4.0,
        allow_direct_cold_start=allow_direct_cold_start,
        low_key_cues=True,
        time_fn=clock or Clock(),
    )


def test_explicit_wake_phrase_activates_engagement() -> None:
    controller = build_attention()

    decision = controller.evaluate('Hey, MoreTea!!! can you help?')

    assert decision.allow_response is True
    assert decision.classification == 'wake'
    assert controller.state is AttentionState.ENGAGED


def test_strong_direct_request_cold_starts_without_name() -> None:
    controller = build_attention()

    decision = controller.evaluate('Where is the printer?')

    assert decision.allow_response is True
    assert decision.classification == 'direct'
    assert controller.state is AttentionState.ENGAGED


def test_standalone_hey_does_not_trigger_cold_start() -> None:
    controller = build_attention()

    decision = controller.evaluate('hey')

    assert decision.allow_response is False
    assert decision.classification == 'ambient'
    assert controller.state is AttentionState.PASSIVE


def test_passive_ambient_speech_is_buffered() -> None:
    controller = build_attention()

    decision = controller.evaluate('We should print the poster after lunch.')

    assert decision.allow_response is False
    assert controller.recent_ambient_texts() == ('we should print the poster after lunch',)


def test_engaged_follow_up_does_not_need_wake_phrase() -> None:
    controller = build_attention()
    controller.evaluate('Hey MoreTea')

    decision = controller.evaluate('tell me where the printer is')

    assert decision.allow_response is True
    assert decision.classification == 'follow_up'
    assert controller.state is AttentionState.ENGAGED


def test_idle_timeout_returns_to_passive() -> None:
    clock = Clock(0.0)
    controller = build_attention(clock=clock)
    controller.evaluate('Hey MoreTea')

    clock.now = 25.0
    decision = controller.evaluate('they are talking about lunch')

    assert decision.allow_response is False
    assert decision.state is AttentionState.PASSIVE
    assert controller.state is AttentionState.PASSIVE


def test_emergency_phrase_bypasses_passive_gating() -> None:
    controller = build_attention()

    decision = controller.evaluate('watch out')

    assert decision.allow_response is True
    assert decision.classification == 'emergency'
    assert controller.state is AttentionState.ENGAGED


def test_agent_stops_ambient_turn_without_prompt_spam() -> None:
    session = FakeSession()
    agent = BareboneAgentHarness(attention=build_attention(), session=session)

    async def run() -> None:
        with pytest.raises(llm.StopResponse):
            await agent.on_user_turn_completed(llm.ChatContext.empty(), SimpleNamespace(content=['hello there']))
        with pytest.raises(llm.StopResponse):
            await agent.on_user_turn_completed(llm.ChatContext.empty(), SimpleNamespace(content=['we should print this later']))

    asyncio.run(run())
    assert session.spoken == []


def test_agent_injects_recent_ambient_context_into_runtime_instructions() -> None:
    clock = Clock(0.0)
    attention = build_attention(clock=clock)
    session = FakeSession()
    agent = BareboneAgentHarness(attention=attention, session=session)

    async def run() -> None:
        with pytest.raises(llm.StopResponse):
            await agent.on_user_turn_completed(
                llm.ChatContext.empty(),
                SimpleNamespace(content=['we were just talking about the printer room']),
            )
        clock.now = 1.0
        await agent.on_user_turn_completed(
            llm.ChatContext.empty(),
            SimpleNamespace(content=['Where is the printer?']),
        )

    asyncio.run(run())
    assert 'we were just talking about the printer room' in agent.instructions
