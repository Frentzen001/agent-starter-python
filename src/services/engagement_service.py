from __future__ import annotations

import re
import time
from dataclasses import dataclass
from enum import Enum

from domain.interaction_domain import InteractionDomain, InteractionState


class UtteranceClass(str, Enum):
    WAKE_PHRASE = 'wake_phrase'
    COMMAND = 'command'
    NOISE = 'noise'
    IGNORE = 'ignore'


@dataclass
class EngagementDecision:
    classification: UtteranceClass
    allow_response: bool
    sleep_prompt: str | None = None
    normalized_text: str = ''
    transitioned_state: InteractionState | None = None
    instructions_changed: bool = False
    immediate_response: str | None = None


class EngagementService:
    def __init__(
        self,
        interaction: InteractionDomain,
        wake_phrases: list[str],
        sleep_prompt: str,
        *,
        wake_greeting: str = 'Hello, I am awake and ready to help.',
        always_awake: bool = False,
    ) -> None:
        self._interaction = interaction
        self._wake_phrases = [self._normalize(item) for item in wake_phrases]
        self._sleep_prompt = sleep_prompt
        self._wake_greeting = wake_greeting
        self._always_awake = always_awake
        if always_awake:
            self._interaction.set_state(InteractionState.ATTENTIVE)

    @property
    def state(self) -> InteractionState:
        return self._interaction.state

    def _normalize(self, text: str) -> str:
        text = text.lower().strip()
        text = re.sub(r'[^a-z0-9\s]', ' ', text)
        return re.sub(r'\s+', ' ', text).strip()

    def _contains_wake_phrase(self, text: str) -> bool:
        normalized = self._normalize(text)
        return any(phrase in normalized for phrase in self._wake_phrases)

    def _remaining_after_wake_phrase(self, text: str) -> str:
        normalized = self._normalize(text)
        for phrase in self._wake_phrases:
            if phrase in normalized:
                return normalized.replace(phrase, '', 1).strip()
        return normalized

    def handle_user_text(self, text: str) -> EngagementDecision:
        normalized = self._normalize(text)
        if not normalized:
            return EngagementDecision(UtteranceClass.NOISE, allow_response=False)

        now = time.monotonic()
        if self._always_awake:
            self._interaction.mark_activity(timestamp=now)
            return EngagementDecision(
                UtteranceClass.COMMAND,
                allow_response=True,
                normalized_text=normalized,
                transitioned_state=self._interaction.state,
                instructions_changed=True,
            )

        if self._contains_wake_phrase(text):
            self._interaction.awaken()
            self._interaction.mark_activity(timestamp=now)
            remaining = self._remaining_after_wake_phrase(text)
            response = self._wake_greeting
            if remaining:
                response = self._wake_greeting + ' I can help with Garage@EEE questions, locations, or tours.'
            return EngagementDecision(
                UtteranceClass.WAKE_PHRASE,
                allow_response=False,
                normalized_text=normalized,
                transitioned_state=self._interaction.state,
                instructions_changed=True,
                immediate_response=response,
            )

        if self._interaction.state == InteractionState.SLEEPING:
            return EngagementDecision(
                UtteranceClass.IGNORE,
                allow_response=False,
                sleep_prompt=self._sleep_prompt,
                normalized_text=normalized,
            )

        self._interaction.mark_activity(timestamp=now)
        if self._interaction.state == InteractionState.ATTENTIVE:
            self._interaction.set_state(InteractionState.CONVERSING, timestamp=now)
        return EngagementDecision(
            UtteranceClass.COMMAND,
            allow_response=True,
            normalized_text=normalized,
            transitioned_state=self._interaction.state,
            instructions_changed=True,
        )

    def mark_guiding(self) -> None:
        self._interaction.set_state(InteractionState.GUIDING)

    def mark_conversing(self) -> None:
        self._interaction.set_state(InteractionState.CONVERSING)

    def mark_blocked(self) -> None:
        self._interaction.set_state(InteractionState.BLOCKED)

    def maybe_sleep(self) -> bool:
        return self._interaction.maybe_sleep()
