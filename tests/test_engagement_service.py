import time

from domain.interaction_domain import InteractionDomain, InteractionState
from services.engagement_service import EngagementService, UtteranceClass


def test_sleeping_state_requires_wake_phrase() -> None:
    interaction = InteractionDomain(idle_timeout_sec=5.0)
    service = EngagementService(interaction, ['hey moretea', 'moretea'], "Say 'Hey MoreTea' to wake me up.")
    decision = service.handle_user_text('hello there')
    assert decision.classification == UtteranceClass.IGNORE
    assert decision.allow_response is False
    assert decision.sleep_prompt is not None


def test_wake_phrase_transitions_to_attentive() -> None:
    interaction = InteractionDomain(idle_timeout_sec=5.0)
    service = EngagementService(interaction, ['hey moretea', 'moretea'], "Say 'Hey MoreTea' to wake me up.")
    decision = service.handle_user_text('hey moretea can you help me')
    assert decision.classification == UtteranceClass.WAKE_PHRASE
    assert decision.allow_response is False
    assert decision.immediate_response is not None
    assert interaction.state == InteractionState.ATTENTIVE


def test_always_awake_mode_bypasses_wake_phrase() -> None:
    interaction = InteractionDomain(idle_timeout_sec=5.0)
    service = EngagementService(interaction, ['moretea'], "Say 'Hey MoreTea' to wake me up.", always_awake=True)
    decision = service.handle_user_text('hello there')
    assert decision.classification == UtteranceClass.COMMAND
    assert decision.allow_response is True
    assert interaction.state == InteractionState.ATTENTIVE


def test_idle_monitor_can_return_to_sleep() -> None:
    interaction = InteractionDomain(idle_timeout_sec=0.01)
    service = EngagementService(interaction, ['moretea'], "Say 'Hey MoreTea' to wake me up.", always_awake=False)
    service.handle_user_text('moretea')
    interaction.last_activity_at = time.monotonic() - 1
    assert service.maybe_sleep() is True
    assert interaction.state == InteractionState.SLEEPING
