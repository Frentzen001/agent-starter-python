# MoreTea Voice Runtime

This repo is the active voice/runtime codebase for MoreTea.

It currently covers:

- LiveKit worker entrypoints
- the barebone `LiveKit -> OpenClaw -> spoken reply` bridge
- voice-side configuration and local model setup
- OpenClaw integration on the OpenClaw PC

For workspace-level context, start with:

- [README.md](/home/frentzen/FYP/README.md)
- [PRD.md](/home/frentzen/FYP/PRD.md)
- [PROGRESS.md](/home/frentzen/FYP/PROGRESS.md)

## Active Voice Paths

### Barebone OpenClaw Bridge

Primary current path:

- entrypoint: [openclaw_barebone.py](/home/frentzen/FYP/agent-starter-python/src/openclaw_barebone.py)
- purpose: minimal LiveKit voice bridge into OpenClaw
- current dev launcher: `./scripts/run_openclaw_barebone.sh`

This path currently keeps:

- LiveKit for voice transport
- OpenClaw as the LLM
- Speaches as the local STT/TTS option

### Legacy MoreTea Worker

Legacy paths still exist for compatibility and older experiments:

- `src/main.py`
- `src/agent.py`

Do not treat them as the primary architecture for the current OpenClaw + MCP direction unless a task explicitly targets legacy behavior.

## Setup

```bash
cd /home/frentzen/FYP/agent-starter-python
uv sync
```

Fresh clone shortcut from the workspace root:

```bash
cd /home/frentzen/FYP
./bootstrap_dev.sh
```

Required environment:

- `LIVEKIT_URL`
- `LIVEKIT_API_KEY`
- `LIVEKIT_API_SECRET`

Start from the template:

```bash
cd /home/frentzen/FYP/agent-starter-python
cp .env.example .env.local
```

## Preferred Development Launch

The preferred development launcher is tmuxinator.

Install the repo-tracked profiles once:

```bash
cd /home/frentzen/FYP
./tmuxinator/install_profiles.sh
```

Robot PC:

```bash
tmuxinator start moretea_robot
```

OpenClaw PC:

```bash
export ROBOT_USER=<robot-user>
export ROBOT_HOST=<robot-ip>
tmuxinator start moretea_voice
```

Tmuxinator details live in [tmuxinator/README.md](/home/frentzen/FYP/tmuxinator/README.md).

## Manual Voice Launch

If you only want the voice-side path manually:

Start Speaches:

```bash
cd /home/frentzen/FYP/agent-starter-python
./scripts/run_speaches.sh
```

Start the barebone worker:

```bash
cd /home/frentzen/FYP/agent-starter-python
./scripts/run_openclaw_barebone.sh
```

Or use the combined helper:

```bash
cd /home/frentzen/FYP/agent-starter-python
./scripts/dev_voice_stack.sh
```

Quick checks:

```bash
curl http://127.0.0.1:8000/v1/models
curl -i -H 'Accept: text/event-stream' http://127.0.0.1:8765/mcp
```

First MCP tool to call after launch:

```text
health
```

## OpenClaw Integration

Current intended integration shape:

- OpenClaw runs on the OpenClaw PC
- OpenClaw consumes robot tools through its native MCP client
- the voice bridge talks to OpenClaw, not directly to ROS

Current robot MCP endpoint from the OpenClaw PC:

```text
http://127.0.0.1:8765/mcp
```

The SSH tunnel is opened with:

```bash
cd /home/frentzen/FYP/moretea-robot-mcp
ROBOT_USER=<robot-user> ROBOT_HOST=<robot-ip> ./scripts/openclaw_tunnel.sh
```

## OpenClaw As LLM Only

If you want the LiveKit agent to keep its existing STT/TTS path but send LLM traffic to OpenClaw, use env vars like:

```env
HOSTED_LLM_BASE_URL=http://<openclaw-host>:18789/v1
HOSTED_LLM_MODEL=openclaw
HOSTED_LLM_API_KEY=<openclaw_token>
HOSTED_LLM_AGENT_ID=main
```

This redirects only the LLM path while leaving STT and TTS on the existing hosted path.

## Barebone Bridge Environment

Typical barebone env:

```env
MORETEA_OPENCLAW_URL=http://<openclaw-host>:18789/v1
MORETEA_OPENCLAW_MODEL=openclaw
MORETEA_OPENCLAW_TOKEN=<token>
MORETEA_OPENCLAW_AGENT_ID=main

MORETEA_WAKE_KEYWORDS=moretea,more tea,hey moretea,hey more tea
MORETEA_SLEEP_PROMPT=Say 'Hey MoreTea' to wake me up.
MORETEA_SLEEP_PROMPT_COOLDOWN_SEC=8
MORETEA_THINKING_CUE_ENABLED=1
MORETEA_THINKING_CUE_VOLUME=0.35
```

### Thinking Cue

The barebone OpenClaw bridge now plays a short nonverbal chirp whenever a user turn passes the attention gate and is forwarded to OpenClaw.

- It plays for accepted turns only.
- It does not speak transcript text or alter the chat context.
- It does not play for ignored ambient speech or sleep prompts.

Current controls:

- `MORETEA_THINKING_CUE_ENABLED=1`
- `MORETEA_THINKING_CUE_VOLUME=0.35`

Current behavior:

- the cue now plays through the local machine speaker in console/dev runs
- it does not require joining a LiveKit room
- if local audio output is unavailable, the worker logs the cue failure and continues normally

Local verification:

- run `./scripts/run_openclaw_barebone.sh`
- speak one accepted turn such as `Where is the printer?`
- confirm you hear the short chirp on the host running the barebone worker

## Local Model Setup

Current recommended local stack:

- `Speaches` for STT and TTS
- `Ollama` if you want a local LLM path

Current default local URLs assume one Speaches server for both STT and TTS:

- `LOCAL_STT_BASE_URL=http://127.0.0.1:8000/v1`
- `LOCAL_TTS_BASE_URL=http://127.0.0.1:8000/v1`

Verify Speaches:

```bash
curl http://127.0.0.1:8000/v1/models
```

Verify Ollama:

```bash
curl http://127.0.0.1:11434/api/tags
```

## Testing

Run voice/runtime tests from this repo:

```bash
cd /home/frentzen/FYP/agent-starter-python
uv run pytest
```

For LiveKit-specific coding guidance, use [AGENTS.md](/home/frentzen/FYP/agent-starter-python/AGENTS.md).
