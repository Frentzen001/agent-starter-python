from __future__ import annotations

import re
from dataclasses import dataclass, field


@dataclass
class MemoryService:
    user_name: str | None = None
    repeat_visitor: bool = False
    preferences: dict[str, str] = field(default_factory=dict)

    def remember_name(self, name: str) -> str | None:
        cleaned = self._clean_name(name)
        if cleaned is None:
            return None
        self.user_name = cleaned
        self.repeat_visitor = True
        return cleaned

    def maybe_capture_name(self, text: str) -> str | None:
        for pattern in (
            r"\bmy name is ([A-Za-z][A-Za-z' -]{0,30})",
            r"\bcall me ([A-Za-z][A-Za-z' -]{0,30})",
        ):
            match = re.search(pattern, text, re.IGNORECASE)
            if match:
                return self.remember_name(match.group(1))
        return None

    def remember_preference(self, key: str, value: str) -> None:
        self.preferences[key] = value.strip()

    def summary(self) -> str | None:
        lines: list[str] = []
        if self.user_name:
            lines.append(f'Known user name: {self.user_name}.')
        mode = self.preferences.get('interaction_mode')
        if mode == 'tour':
            lines.append('Preference hint: the user currently wants tour or navigation help.')
        elif mode == 'faq':
            lines.append('Preference hint: the user currently prefers question-and-answer help.')
        elif mode:
            lines.append(f'Preference hint: {mode}.')
        if self.repeat_visitor:
            lines.append('Treat the user as a returning visitor within this session.')
        return ' '.join(lines) or None

    def _clean_name(self, name: str) -> str | None:
        cleaned = ' '.join(name.strip().split())
        if not cleaned:
            return None
        words = [word for word in cleaned.split(' ') if word]
        if len(words) > 2:
            words = words[:2]
        normalized = ' '.join(word[:1].upper() + word[1:].lower() for word in words)
        return normalized if normalized else None
