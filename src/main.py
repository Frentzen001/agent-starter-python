from __future__ import annotations

import logging
import os

from dotenv import load_dotenv
from livekit.agents import AgentServer, JobContext, JobProcess, cli
from livekit.plugins import silero

from runtime.session_runtime import SessionRuntime

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)-8s %(name)s - %(message)s',
    datefmt='%H:%M:%S',
)

load_dotenv('.env.local')

server = AgentServer()


def _env_flag(name: str, *, default: bool = False) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def prewarm(proc: JobProcess) -> None:
    proc.userdata['vad'] = silero.VAD.load()


server.setup_fnc = prewarm


@server.rtc_session(agent_name='my-agent')
async def my_agent(ctx: JobContext) -> None:
    await ctx.connect()
    runtime = SessionRuntime(job_ctx=ctx, always_awake=_env_flag('MORETEA_ALWAYS_AWAKE', default=False))
    await runtime.start()


if __name__ == '__main__':
    cli.run_app(server)
