import pytest

from domain.tour_domain import NavigationOutcome, TourStop
from services.navigation_service import NavigationService
from services.telemetry_service import TelemetryService


class _FakeConnector:
    def __init__(self, raw: str):
        self.raw = raw
        self.is_navigating = False
        self.distance_remaining = 1.2
        self.recovery_count = 1
        self.replan_count = 2
        self.last_event_note = 'Taking a different route.'
        self.cancelled = False

    async def navigate_to_pose(self, x: float, y: float, ow: float) -> str:
        return self.raw

    async def cancel_navigation(self) -> None:
        self.cancelled = True


@pytest.mark.asyncio
async def test_navigation_service_maps_successful_result() -> None:
    service = NavigationService(_FakeConnector('succeeded'), TelemetryService())
    stop = TourStop(id='entrance', name='Entrance', aliases=(), x=0.0, y=0.0, ow=1.0, narration='Start')
    result = await service.navigate_to_stop(stop)
    assert result.outcome == NavigationOutcome.SUCCEEDED
    assert result.distance_remaining == 1.2
    assert result.recovery_count == 1
    assert result.replan_count == 2
    assert result.event_note == 'Taking a different route.'


@pytest.mark.asyncio
async def test_navigation_service_reports_unavailable_without_connector() -> None:
    service = NavigationService(None, TelemetryService())
    stop = TourStop(id='entrance', name='Entrance', aliases=(), x=0.0, y=0.0, ow=1.0, narration='Start')
    result = await service.navigate_to_stop(stop)
    assert result.outcome == NavigationOutcome.UNAVAILABLE
