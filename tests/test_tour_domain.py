from domain.tour_domain import NavigationOutcome, NavigationResult, TourDomain, TourStop, TourState


def _stops() -> tuple[TourStop, ...]:
    return (
        TourStop(id='entrance', name='Entrance', aliases=(), x=0.0, y=0.0, ow=1.0, narration='Start'),
        TourStop(id='lab', name='Lab', aliases=('electronics',), x=1.0, y=1.0, ow=1.0, narration='Lab'),
        TourStop(id='exit', name='Exit', aliases=(), x=2.0, y=2.0, ow=1.0, narration='End'),
    )


def test_start_tour_targets_first_stop() -> None:
    domain = TourDomain(_stops())
    decision = domain.start_tour()
    assert decision.target_stop is not None
    assert decision.target_stop.id == 'entrance'
    assert decision.handoff_to_tour_agent is True
    assert domain.state == TourState.NAVIGATING


def test_successful_navigation_advances_progress() -> None:
    domain = TourDomain(_stops())
    domain.start_tour()
    follow_up = domain.handle_navigation_result(NavigationResult(outcome=NavigationOutcome.SUCCEEDED))
    assert domain.last_completed_stop_id == 'entrance'
    assert domain.state == TourState.PAUSED
    assert 'Next stop is Lab' in follow_up.speech


def test_canceled_navigation_keeps_pending_stop() -> None:
    domain = TourDomain(_stops())
    domain.start_tour()
    follow_up = domain.handle_navigation_result(NavigationResult(outcome=NavigationOutcome.CANCELED))
    assert domain.state == TourState.PAUSED
    assert domain.pending_stop_id == 'entrance'
    assert 'canceled' in follow_up.speech.lower()


def test_failed_navigation_retries_once_then_skips() -> None:
    domain = TourDomain(_stops())
    decision = domain.start_tour()
    retry = domain.handle_navigation_result(
        NavigationResult(outcome=NavigationOutcome.FAILED_RECOVERABLE, detail='failed')
    )
    assert retry.target_stop == decision.target_stop
    assert 'retry once' in retry.speech.lower()

    skip = domain.handle_navigation_result(
        NavigationResult(outcome=NavigationOutcome.FAILED_RECOVERABLE, detail='failed again')
    )
    assert domain.pending_stop_id == 'lab'
    assert domain.skipped_stop_ids == ['entrance']
    assert skip.target_stop is not None
    assert skip.target_stop.id == 'lab'
    assert 'skipping it' in skip.speech.lower()


def test_unavailable_navigation_blocks_tour() -> None:
    domain = TourDomain(_stops())
    domain.start_tour()
    follow_up = domain.handle_navigation_result(NavigationResult(outcome=NavigationOutcome.UNAVAILABLE))
    assert domain.state == TourState.BLOCKED
    assert follow_up.handoff_to_concierge is True
