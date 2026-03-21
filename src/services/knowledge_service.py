from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from domain.interaction_domain import InteractionState
from domain.safety_domain import SafetyState
from domain.tour_domain import TourStatus, TourStop
from knowledge import GARAGE_KNOWLEDGE

NL = chr(10)


@dataclass(frozen=True)
class PersonaConfig:
    name: str
    role: str
    voice_style: str
    wake_phrases: tuple[str, ...]
    greetings: tuple[str, ...]
    sleep_prompt: str
    tour_completion_prompt: str
    system_principles: tuple[str, ...]


class KnowledgeService:
    MAX_INSTRUCTION_CHARS = 24_000

    def __init__(self, *, content_dir: Path | None = None) -> None:
        self._content_dir = content_dir or Path(__file__).resolve().parent.parent / 'content'
        self._persona_raw = self._load_yaml('robot_persona.yaml')
        self._policies_raw = self._load_yaml('garage_policies.yaml')
        self._faq_raw = self._load_yaml('garage_faq.yaml')

    def _load_yaml(self, name: str) -> dict[str, Any]:
        path = self._content_dir / name
        with path.open('r', encoding='utf-8') as fh:
            data = yaml.safe_load(fh) or {}
        if not isinstance(data, dict):
            raise ValueError(f'{name} must contain a mapping at the top level')
        return data

    @property
    def persona(self) -> PersonaConfig:
        data = self._persona_raw
        return PersonaConfig(
            name=data['name'],
            role=data['role'],
            voice_style=data['voice_style'],
            wake_phrases=tuple(data.get('wake_phrases', [])),
            greetings=tuple(data.get('greeting', [])),
            sleep_prompt=data['sleep_prompt'],
            tour_completion_prompt=data['tour_completion_prompt'],
            system_principles=tuple(data.get('system_principles', [])),
        )

    @property
    def faq_entries(self) -> list[dict[str, Any]]:
        return list(self._faq_raw.get('faqs', []))

    @property
    def policies(self) -> tuple[str, ...]:
        return tuple(self._policies_raw.get('rules', []))

    def faq_prompt_section(self) -> str:
        lines = ['## Curated Garage FAQ']
        for item in self.faq_entries:
            lines.append(f"- Q: {item['question']}")
            lines.append(f"  A: {item['answer']}")
        return NL.join(lines)

    def stop_prompt_section(self, stops: tuple[TourStop, ...]) -> str:
        lines = ['## Tour Stops']
        for idx, stop in enumerate(stops, start=1):
            lines.append(f'- {idx}. {stop.name}: {stop.narration}')
        return NL.join(lines)

    def current_rules_section(
        self,
        interaction_state: InteractionState,
        tour_status: TourStatus,
        safety_state: SafetyState,
        safety_reason: str | None,
    ) -> str:
        lines = ['## Runtime Rules']
        lines.extend(f'- {rule}' for rule in self.policies)
        lines.append(f'- Current interaction state: {interaction_state.value}.')
        lines.append(f'- Current tour state: {tour_status.state.value}.')
        lines.append(f'- Current safety state: {safety_state.value}.')
        if safety_reason:
            lines.append(f'- Safety note: {safety_reason}')
        if tour_status.current_stop_id:
            lines.append(f'- Active tour stop id: {tour_status.current_stop_id}.')
        if tour_status.next_stop_id:
            lines.append(f'- Next tour stop id: {tour_status.next_stop_id}.')
        if tour_status.last_completed_stop_id:
            lines.append(f'- Last completed stop id: {tour_status.last_completed_stop_id}.')
        return NL.join(lines)

    def memory_section(self, memory_summary: str | None) -> str | None:
        if not memory_summary:
            return None
        return '## Session Memory' + NL + memory_summary

    def build_agent_instructions(
        self,
        *,
        agent_role: str,
        interaction_state: InteractionState,
        tour_status: TourStatus,
        stops: tuple[TourStop, ...],
        safety_state: SafetyState,
        safety_reason: str | None,
        memory_summary: str | None,
    ) -> str:
        persona = self.persona
        role_block = {
            'concierge': (
                'You are the always-available concierge for Garage@EEE. '
                'Greet users, answer Garage questions, start tours, and route people to locations.'
            ),
            'tour': (
                'You are the active tour guide. Keep the tour moving, explain each stop, '
                'handle interruptions calmly, and ask whether the user wants to continue.'
            ),
        }[agent_role]
        sleep_rule = (
            f'If the robot is sleeping, do not continue a full conversation. Tell the user: {persona.sleep_prompt}'
            if interaction_state == InteractionState.SLEEPING
            else 'The robot is awake. Be concise, warm, and proactive.'
        )
        principles = NL.join(f'- {item}' for item in persona.system_principles)
        sections = [
            f'You are {persona.name}, a {persona.role} in Garage@EEE. Your style is {persona.voice_style}.',
            role_block,
            sleep_rule,
            '## Personality Principles' + NL + principles,
            self.current_rules_section(interaction_state, tour_status, safety_state, safety_reason),
            self.stop_prompt_section(stops),
            '## Garage Knowledge' + NL + GARAGE_KNOWLEDGE,
            self.faq_prompt_section(),
        ]
        memory_block = self.memory_section(memory_summary)
        if memory_block:
            sections.insert(4, memory_block)
        return (NL + NL).join(sections)

    def lookup_faq(self, query: str) -> str | None:
        norm = self._normalize(query)
        if not norm:
            return None
        query_tokens = set(self._tokenize(norm))
        best_answer = None
        best_score = 0
        for item in self.faq_entries:
            score = 0
            question_norm = self._normalize(item['question'])
            if question_norm and question_norm in norm:
                score += 5
            for keyword in item.get('keywords', []):
                key = self._normalize(keyword)
                if not key:
                    continue
                key_tokens = set(self._tokenize(key))
                if len(key_tokens) > 1 and key in norm:
                    score += 3
                elif key_tokens and key_tokens.issubset(query_tokens):
                    score += 2
            if score > best_score:
                best_score = score
                best_answer = item['answer']
        return best_answer if best_score >= 2 else None

    def instruction_within_budget(self, instructions: str) -> bool:
        return len(instructions) <= self.MAX_INSTRUCTION_CHARS

    def _normalize(self, text: str) -> str:
        lowered = text.lower().strip()
        lowered = re.sub(r'[^a-z0-9\s]', ' ', lowered)
        return re.sub(r'\s+', ' ', lowered).strip()

    def _tokenize(self, text: str) -> list[str]:
        return [token for token in text.split(' ') if token]
