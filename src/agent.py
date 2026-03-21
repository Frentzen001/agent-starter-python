from __future__ import annotations

from agent.concierge_agent import ConciergeAgent as Assistant
from main import my_agent, prewarm, server

__all__ = ['Assistant', 'my_agent', 'prewarm', 'server']

if __name__ == '__main__':
    from livekit.agents import cli

    cli.run_app(server)
