from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from typing import Any


@dataclass
class TelemetryService:
    logger: logging.Logger = field(default_factory=lambda: logging.getLogger('moretea.telemetry'))

    def event(self, name: str, **fields: Any) -> None:
        payload = {'event': name, **fields}
        self.logger.info(json.dumps(payload, default=str, sort_keys=True))

    def startup_health(self, report: Any) -> None:
        if hasattr(report, 'as_payload'):
            payload = report.as_payload()
        else:
            payload = report
        self.event('startup_health', **payload)
