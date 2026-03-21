from __future__ import annotations

from domain.tour_domain import NavigationOutcome, NavigationResult, TourStop


class NavigationService:
    def __init__(self, connector, telemetry) -> None:
        self._connector = connector
        self._telemetry = telemetry

    @property
    def available(self) -> bool:
        return self._connector is not None

    async def navigate_to_stop(self, stop: TourStop) -> NavigationResult:
        if self._connector is None:
            return NavigationResult(
                outcome=NavigationOutcome.UNAVAILABLE,
                detail='ROS 2 connector not available.',
            )
        raw = await self._connector.navigate_to_pose(stop.x, stop.y, stop.ow)
        result = self._map_result(raw)
        result.distance_remaining = self.distance_remaining
        result.recovery_count = getattr(self._connector, 'recovery_count', 0)
        result.replan_count = getattr(self._connector, 'replan_count', 0)
        result.event_note = getattr(self._connector, 'last_event_note', None)
        self._telemetry.event(
            'navigation_result',
            stop_id=stop.id,
            outcome=result.outcome.value,
            detail=result.detail,
            recovery_count=result.recovery_count,
            replan_count=result.replan_count,
        )
        return result

    def _map_result(self, raw: str) -> NavigationResult:
        normalized = raw.lower().strip()
        if normalized == 'succeeded':
            return NavigationResult(outcome=NavigationOutcome.SUCCEEDED, detail=raw)
        if normalized == 'canceled':
            return NavigationResult(outcome=NavigationOutcome.CANCELED, detail=raw)
        if 'unavailable' in normalized:
            return NavigationResult(outcome=NavigationOutcome.UNAVAILABLE, detail=raw)
        if 'recoverable' in normalized:
            return NavigationResult(outcome=NavigationOutcome.FAILED_RECOVERABLE, detail=raw)
        if normalized == 'failed':
            return NavigationResult(outcome=NavigationOutcome.FAILED_RECOVERABLE, detail=raw)
        return NavigationResult(outcome=NavigationOutcome.FAILED_UNRECOVERABLE, detail=raw)

    async def cancel(self) -> None:
        if self._connector is not None:
            await self._connector.cancel_navigation()

    @property
    def is_navigating(self) -> bool:
        return False if self._connector is None else self._connector.is_navigating

    @property
    def distance_remaining(self) -> float | None:
        return None if self._connector is None else self._connector.distance_remaining

    def status_text(self) -> str:
        if self._connector is None or not self._connector.is_navigating:
            return 'The robot is not currently navigating.'
        if self._connector.distance_remaining is None:
            return 'The robot is navigating, but distance feedback is not available yet.'
        return f'Approximately {self._connector.distance_remaining:.1f} metres remain.'
