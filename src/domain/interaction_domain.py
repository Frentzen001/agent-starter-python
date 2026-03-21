from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum


class InteractionState(str, Enum):
    SLEEPING = "sleeping"
    ATTENTIVE = "attentive"
    CONVERSING = "conversing"
    GUIDING = "guiding"
    BLOCKED = "blocked"


@dataclass
class InteractionDomain:
    idle_timeout_sec: float = 5.0
    state: InteractionState = InteractionState.SLEEPING
    last_activity_at: float | None = None

    def set_state(self, state: InteractionState, *, timestamp: float | None = None) -> None:
        self.state = state
        self.last_activity_at = timestamp or time.monotonic()

    def mark_activity(self, *, timestamp: float | None = None) -> None:
        self.last_activity_at = timestamp or time.monotonic()
        if self.state == InteractionState.SLEEPING:
            self.state = InteractionState.ATTENTIVE

    def awaken(self) -> None:
        self.set_state(InteractionState.ATTENTIVE)

    def maybe_sleep(self, *, now: float | None = None) -> bool:
        if self.state == InteractionState.SLEEPING or self.last_activity_at is None:
            return False
        current = now or time.monotonic()
        if current - self.last_activity_at >= self.idle_timeout_sec:
            self.state = InteractionState.SLEEPING
            self.last_activity_at = current
            return True
        return False
