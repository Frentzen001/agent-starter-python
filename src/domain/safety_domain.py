from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class SafetyState(str, Enum):
    NORMAL = "normal"
    BLOCKED = "blocked"
    DEGRADED = "degraded"


@dataclass
class SafetyDomain:
    state: SafetyState = SafetyState.NORMAL
    reason: str | None = None

    @property
    def motion_allowed(self) -> bool:
        return self.state == SafetyState.NORMAL

    def block(self, reason: str) -> None:
        self.state = SafetyState.BLOCKED
        self.reason = reason

    def degrade(self, reason: str) -> None:
        self.state = SafetyState.DEGRADED
        self.reason = reason

    def restore(self) -> None:
        self.state = SafetyState.NORMAL
        self.reason = None
