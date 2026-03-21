from __future__ import annotations

import logging

from domain.interaction_domain import InteractionState
from domain.tour_domain import NavigationOutcome

try:
    from std_msgs.msg import Int32
except ImportError:
    Int32 = None


class AffectService:
    EMOTION_MAP = {
        'neutral': 0,
        'happy': 1,
        'sad': 2,
        'angry': 3,
        'confused': 4,
        'shocked': 5,
        'love': 6,
        'shy': 7,
    }

    STATE_TO_EMOTION = {
        InteractionState.SLEEPING: 'neutral',
        InteractionState.ATTENTIVE: 'love',
        InteractionState.CONVERSING: 'happy',
        InteractionState.GUIDING: 'happy',
        InteractionState.BLOCKED: 'confused',
    }

    def __init__(self, connector) -> None:
        self._connector = connector
        self._logger = logging.getLogger('moretea.affect')

    def set_emotion(self, emotion: str) -> None:
        if self._connector is None or Int32 is None:
            return
        value = self.EMOTION_MAP.get(emotion, 0)
        self._logger.info('eye expression -> %s (%s)', emotion, value)
        self._connector.publish('/eye_expression', Int32, {'data': value})

    def apply_interaction_state(self, state: InteractionState) -> None:
        self.set_emotion(self.STATE_TO_EMOTION.get(state, 'neutral'))

    def apply_navigation_outcome(self, outcome: NavigationOutcome) -> None:
        mapping = {
            NavigationOutcome.SUCCEEDED: 'happy',
            NavigationOutcome.CANCELED: 'neutral',
            NavigationOutcome.FAILED_RECOVERABLE: 'confused',
            NavigationOutcome.FAILED_UNRECOVERABLE: 'sad',
            NavigationOutcome.UNAVAILABLE: 'sad',
        }
        self.set_emotion(mapping.get(outcome, 'neutral'))
