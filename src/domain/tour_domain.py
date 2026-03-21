from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class NavigationOutcome(str, Enum):
    SUCCEEDED = "succeeded"
    CANCELED = "canceled"
    FAILED_RECOVERABLE = "failed_recoverable"
    FAILED_UNRECOVERABLE = "failed_unrecoverable"
    UNAVAILABLE = "unavailable"


class TourState(str, Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    PAUSED = "paused"
    COMPLETED = "completed"
    BLOCKED = "blocked"


@dataclass(frozen=True)
class TourStop:
    id: str
    name: str
    aliases: tuple[str, ...]
    x: float
    y: float
    ow: float
    narration: str
    faq_tags: tuple[str, ...] = ()
    escort_only: bool = False

    def matches(self, query: str) -> bool:
        norm = query.lower().strip()
        candidates = [self.name, *self.aliases, self.id.replace('_', ' ')]
        return any(norm in item.lower() or item.lower() in norm for item in candidates)


@dataclass
class NavigationResult:
    outcome: NavigationOutcome
    detail: str = ""
    distance_remaining: float | None = None
    recovery_count: int = 0
    replan_count: int = 0
    event_note: str | None = None


@dataclass
class TourStatus:
    state: TourState
    current_stop_id: str | None
    next_stop_id: str | None
    last_completed_stop_id: str | None
    skipped_stop_ids: tuple[str, ...]
    retry_count: int
    last_navigation_outcome: NavigationOutcome | None


@dataclass
class TourDecision:
    speech: str
    target_stop: TourStop | None = None
    handoff_to_tour_agent: bool = False
    handoff_to_concierge: bool = False


@dataclass
class TourDomain:
    stops: tuple[TourStop, ...]
    max_retries_per_stop: int = 1
    state: TourState = TourState.IDLE
    current_index: int = 0
    pending_stop_id: str | None = None
    last_completed_stop_id: str | None = None
    skipped_stop_ids: list[str] = field(default_factory=list)
    retry_counts: dict[str, int] = field(default_factory=dict)
    last_navigation_outcome: NavigationOutcome | None = None

    def __post_init__(self) -> None:
        self._stop_lookup = {stop.id: stop for stop in self.stops}

    def _stop_by_id(self, stop_id: str | None) -> TourStop | None:
        return None if stop_id is None else self._stop_lookup.get(stop_id)

    def _index_for_stop(self, stop_id: str) -> int:
        for idx, stop in enumerate(self.stops):
            if stop.id == stop_id:
                return idx
        raise KeyError(stop_id)

    def _next_stop(self) -> TourStop | None:
        if self.current_index >= len(self.stops):
            return None
        return self.stops[self.current_index]

    def find_stop(self, query: str) -> TourStop | None:
        for stop in self.stops:
            if stop.matches(query):
                return stop
        return None

    def start_tour(self) -> TourDecision:
        self.state = TourState.NAVIGATING
        self.current_index = 0
        self.pending_stop_id = self.stops[0].id if self.stops else None
        self.last_completed_stop_id = None
        self.skipped_stop_ids.clear()
        self.retry_counts.clear()
        self.last_navigation_outcome = None
        stop = self._stop_by_id(self.pending_stop_id)
        if stop is None:
            self.state = TourState.BLOCKED
            return TourDecision("No tour stops are configured, so I cannot start the tour yet.")
        return TourDecision(
            speech=f"Starting the Garage@EEE tour. Our first stop is {stop.name}.",
            target_stop=stop,
            handoff_to_tour_agent=True,
        )

    def go_to_location(self, query: str) -> TourDecision:
        stop = self.find_stop(query)
        if stop is None:
            available = ", ".join(item.name for item in self.stops)
            return TourDecision(f"I could not find '{query}'. Available locations are: {available}.")
        self.pending_stop_id = stop.id
        self.current_index = self._index_for_stop(stop.id)
        self.state = TourState.NAVIGATING
        return TourDecision(
            speech=f"Heading to {stop.name} now.",
            target_stop=stop,
            handoff_to_tour_agent=True,
        )

    def pause(self) -> TourDecision:
        self.state = TourState.PAUSED
        return TourDecision("Tour paused. The robot has stopped safely.")

    def resume(self) -> TourDecision:
        stop = self._stop_by_id(self.pending_stop_id) or self._next_stop()
        if stop is None:
            self.state = TourState.COMPLETED
            return TourDecision("The tour is already complete.", handoff_to_concierge=True)
        self.pending_stop_id = stop.id
        self.state = TourState.NAVIGATING
        return TourDecision(
            speech=f"Continuing the tour to {stop.name}.",
            target_stop=stop,
            handoff_to_tour_agent=True,
        )

    def emergency_stop(self) -> TourDecision:
        self.state = TourState.PAUSED
        return TourDecision("Emergency stop acknowledged. The robot has stopped immediately.")

    def handle_navigation_result(self, result: NavigationResult) -> TourDecision:
        self.last_navigation_outcome = result.outcome
        pending = self._stop_by_id(self.pending_stop_id)
        if pending is None:
            self.state = TourState.IDLE
            return TourDecision("Navigation finished, but there was no active stop to update.")

        if result.outcome == NavigationOutcome.SUCCEEDED:
            self.last_completed_stop_id = pending.id
            self.current_index = self._index_for_stop(pending.id) + 1
            self.pending_stop_id = None
            next_stop = self._next_stop()
            if next_stop is None:
                self.state = TourState.COMPLETED
                return TourDecision(
                    speech=f"Arrived at {pending.name}. That was the final stop of the tour.",
                    handoff_to_concierge=True,
                )
            self.state = TourState.PAUSED
            return TourDecision(
                speech=f"Arrived at {pending.name}. Next stop is {next_stop.name}. Ask me to continue when you are ready.",
            )

        if result.outcome == NavigationOutcome.CANCELED:
            self.state = TourState.PAUSED
            return TourDecision(f"Navigation to {pending.name} was canceled. I can continue there when you are ready.")

        if result.outcome == NavigationOutcome.UNAVAILABLE:
            self.state = TourState.BLOCKED
            return TourDecision(
                f"Navigation is unavailable, so I cannot move to {pending.name}. I can still answer questions as a stationary concierge.",
                handoff_to_concierge=True,
            )

        retries = self.retry_counts.get(pending.id, 0)
        if retries < self.max_retries_per_stop:
            self.retry_counts[pending.id] = retries + 1
            self.state = TourState.NAVIGATING
            return TourDecision(
                speech=f"I could not reach {pending.name} on the first attempt, so I will retry once.",
                target_stop=pending,
                handoff_to_tour_agent=True,
            )

        self.skipped_stop_ids.append(pending.id)
        self.current_index = self._index_for_stop(pending.id) + 1
        next_stop = self._next_stop()
        if next_stop is None:
            self.pending_stop_id = None
            self.state = TourState.COMPLETED
            return TourDecision(
                speech=f"I could not reach {pending.name}, and there are no more stops left. The tour is complete.",
                handoff_to_concierge=True,
            )

        self.pending_stop_id = next_stop.id
        self.state = TourState.NAVIGATING
        return TourDecision(
            speech=(
                f"I still could not reach {pending.name}, so I am skipping it for now. "
                f"Continuing to {next_stop.name}."
            ),
            target_stop=next_stop,
            handoff_to_tour_agent=True,
        )

    def status(self) -> TourStatus:
        current_stop = self._stop_by_id(self.pending_stop_id)
        next_stop = current_stop or self._next_stop()
        retry_count = self.retry_counts.get(self.pending_stop_id or '', 0)
        return TourStatus(
            state=self.state,
            current_stop_id=self.pending_stop_id,
            next_stop_id=next_stop.id if next_stop else None,
            last_completed_stop_id=self.last_completed_stop_id,
            skipped_stop_ids=tuple(self.skipped_stop_ids),
            retry_count=retry_count,
            last_navigation_outcome=self.last_navigation_outcome,
        )
