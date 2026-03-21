from __future__ import annotations

from domain.interaction_domain import InteractionState
from domain.safety_domain import SafetyState
from domain.tour_domain import TourStatus, TourStop, TourState
from services.knowledge_service import KnowledgeService


def _stops() -> tuple[TourStop, ...]:
    return (
        TourStop(id='entrance', name='Entrance', aliases=('front door',), x=0.0, y=0.0, ow=1.0, narration='Start here.'),
        TourStop(id='lab', name='Lab', aliases=('electronics',), x=1.0, y=1.0, ow=1.0, narration='Build things here.'),
    )


def _status() -> TourStatus:
    return TourStatus(
        state=TourState.IDLE,
        current_stop_id=None,
        next_stop_id='entrance',
        last_completed_stop_id=None,
        skipped_stop_ids=(),
        retry_count=0,
        last_navigation_outcome=None,
    )


def test_prompt_stays_within_budget() -> None:
    service = KnowledgeService()
    instructions = service.build_agent_instructions(
        agent_role='concierge',
        interaction_state=InteractionState.ATTENTIVE,
        tour_status=_status(),
        stops=_stops(),
        safety_state=SafetyState.NORMAL,
        safety_reason=None,
        memory_summary='Known user name: Alex.',
    )
    assert service.instruction_within_budget(instructions)
    assert len(instructions) <= service.MAX_INSTRUCTION_CHARS


def test_lookup_faq_ignores_weak_overlap() -> None:
    service = KnowledgeService()
    assert service.lookup_faq('Tell me something unrelated about lunch and weather.') is None
