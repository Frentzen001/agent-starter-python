# MoreTea Developer Guide

MoreTea is a Garage@EEE pet robot assistant built on LiveKit Agents, ROS 2, and Nav2. This README documents the current implementation, entrypoints, commands, local-model setup, and extension points for future developers.

## Entry Points
- `src/main.py`: primary LiveKit worker entrypoint.
- `src/agent.py`: compatibility entrypoint preserved for existing commands, Docker usage, and taskfile usage.
- `src/agent/__init__.py`: compatibility export surface for `from agent import Assistant`.
- `src/ros2_connector.py`: main ROS2 connector implementation used by compatibility tests.
- `src/integrations/ros2_connector.py`: wrapper import surface for the refactored layout.

## Architecture

```text
src/
  main.py
  agent.py
  agent/
    __init__.py
    base_agent.py
    concierge_agent.py
    tour_agent.py
  runtime/
    model_config.py
    session_runtime.py
  domain/
    interaction_domain.py
    safety_domain.py
    tour_domain.py
  services/
    navigation_service.py
    knowledge_service.py
    engagement_service.py
    affect_service.py
    memory_service.py
    telemetry_service.py
  integrations/
    ros2_connector.py
  content/
    robot_persona.yaml
    garage_policies.yaml
    garage_faq.yaml
    tour_stops.yaml
  knowledge.py
  ros2_connector.py
```

## Design Rules
- The LLM handles intent and phrasing, not robot safety or state transitions.
- Garage knowledge is prompt-injected at startup because the token footprint is small.
- Robot control and tour state are deterministic services and domain models.
- Wake behaviour is implemented in software, not with RESpeaker firmware features.
- Existing script and import compatibility is intentionally preserved.

## Content Files
- `src/content/robot_persona.yaml`: wake phrases, greeting style, and persona rules.
- `src/content/garage_policies.yaml`: runtime behaviour and safety policy snippets.
- `src/content/garage_faq.yaml`: curated FAQ answers from committee members.
- `src/content/tour_stops.yaml`: stop names, aliases, coordinates, and narration.

## Setup

```bash
cd agent-starter-python
uv sync
```

Configure `.env.local` with at least:
- `LIVEKIT_URL`
- `LIVEKIT_API_KEY`
- `LIVEKIT_API_SECRET`

Optional runtime flags:
- `MORETEA_ALWAYS_AWAKE=1` to disable sleep gating for demos or debugging.
- `USE_LOCAL_MODELS=1` to switch STT, LLM, and TTS from LiveKit Inference to your own local services.
- `LOCAL_MODEL_FALLBACK_TO_LIVEKIT=1` to automatically fall back to hosted models when a local service is unavailable at startup.

## Local Model Setup

Recommended starter stack for your laptop:
- `Ollama` for the LLM
- `Speaches` for both STT and TTS

Why this stack:
- Ollama is already directly supported by the LiveKit OpenAI plugin for the LLM.
- Speaches is an OpenAI-compatible server for both speech-to-text and text-to-speech, using `faster-whisper` for STT and `Kokoro` or `Piper` for TTS.
- This lets you keep one local server for STT and TTS instead of managing two unrelated APIs.

If your local speech services are not ready yet, keep `LOCAL_MODEL_FALLBACK_TO_LIVEKIT=1` so startup can fall back to hosted models.

### 1. Start Ollama for the LLM

Install Ollama on your machine, then start it:

```bash
ollama serve
```

In another terminal, pull the recommended model:

```bash
ollama pull qwen2.5:7b-instruct
```

Verify Ollama is reachable:

```bash
curl http://127.0.0.1:11434/api/tags
```

Expected result: a JSON response listing installed models.

### 2. Start Speaches for local STT and TTS

The simplest path is Docker with GPU support. Create a local directory for model cache first:

```bash
mkdir -p ~/.cache/huggingface
```

Start Speaches on port `8000`:

```bash
docker run --gpus all -p 8000:8000 \
  -v ~/.cache/huggingface:/root/.cache/huggingface \
  ghcr.io/speaches-ai/speaches:latest-cuda
```

If Docker GPU passthrough is not working yet, you can still try the same container without `--gpus all`, but STT and TTS latency will be worse.

Verify the server is up:

```bash
curl http://127.0.0.1:8000/v1/models
```

Expected result: a JSON model list from Speaches.

### 3. Download the STT and TTS models into Speaches

After the Speaches container is running, open another terminal and download one STT model and one TTS model:

```bash
uvx speaches-cli model download Systran/faster-distil-whisper-small.en
uvx speaches-cli model download speaches-ai/Kokoro-82M-v1.0-ONNX
```

Verify they are available:

```bash
curl http://127.0.0.1:8000/v1/models
```

### 4. Use one Speaches base URL for both STT and TTS

Because Speaches serves both APIs on the same port, point both STT and TTS to `http://127.0.0.1:8000/v1`.

Use this in `.env.local`:

```env
USE_LOCAL_MODELS=1
LOCAL_MODEL_FALLBACK_TO_LIVEKIT=1
MORETEA_ALWAYS_AWAKE=0

LOCAL_LLM_BASE_URL=http://127.0.0.1:11434/v1
LOCAL_LLM_MODEL=qwen2.5:7b-instruct
LOCAL_LLM_API_KEY=ollama
LOCAL_LLM_HEALTH_URL=http://127.0.0.1:11434/api/tags

LOCAL_STT_BASE_URL=http://127.0.0.1:8000/v1
LOCAL_STT_MODEL=Systran/faster-distil-whisper-small.en
LOCAL_STT_LANGUAGE=en
LOCAL_STT_API_KEY=local-stt
LOCAL_STT_HEALTH_URL=http://127.0.0.1:8000/v1/models

LOCAL_TTS_BASE_URL=http://127.0.0.1:8000/v1
LOCAL_TTS_MODEL=speaches-ai/Kokoro-82M-v1.0-ONNX
LOCAL_TTS_VOICE=af_heart
LOCAL_TTS_API_KEY=local-tts
LOCAL_TTS_HEALTH_URL=http://127.0.0.1:8000/v1/models
LOCAL_TTS_INSTRUCTIONS=Speak in a friendly and concise tone.
LOCAL_TTS_RESPONSE_FORMAT=pcm
```

You can keep your existing `LIVEKIT_URL`, `LIVEKIT_API_KEY`, and `LIVEKIT_API_SECRET` in the same file for transport and hosted fallback.

### 5. Full example `.env.local`

```env
LIVEKIT_URL=wss://your-livekit-instance
LIVEKIT_API_KEY=your_livekit_key
LIVEKIT_API_SECRET=your_livekit_secret

USE_LOCAL_MODELS=1
LOCAL_MODEL_FALLBACK_TO_LIVEKIT=1
MORETEA_ALWAYS_AWAKE=0

LOCAL_LLM_BASE_URL=http://127.0.0.1:11434/v1
LOCAL_LLM_MODEL=qwen2.5:7b-instruct
LOCAL_LLM_API_KEY=ollama
LOCAL_LLM_HEALTH_URL=http://127.0.0.1:11434/api/tags

LOCAL_STT_BASE_URL=http://127.0.0.1:8000/v1
LOCAL_STT_MODEL=Systran/faster-distil-whisper-small.en
LOCAL_STT_LANGUAGE=en
LOCAL_STT_API_KEY=local-stt
LOCAL_STT_HEALTH_URL=http://127.0.0.1:8000/v1/models

LOCAL_TTS_BASE_URL=http://127.0.0.1:8000/v1
LOCAL_TTS_MODEL=speaches-ai/Kokoro-82M-v1.0-ONNX
LOCAL_TTS_VOICE=af_heart
LOCAL_TTS_API_KEY=local-tts
LOCAL_TTS_HEALTH_URL=http://127.0.0.1:8000/v1/models
LOCAL_TTS_INSTRUCTIONS=Speak in a friendly and concise tone.
LOCAL_TTS_RESPONSE_FORMAT=pcm
```

### 6. Start the agent with local models

Download the local LiveKit helper files once:

```bash
uv run python src/agent.py download-files
```

Run the agent:

```bash
uv run python src/agent.py dev
```

At startup, the runtime will:
- validate the local model URLs
- probe the local health endpoints
- select the local stack if all checks pass
- fall back to hosted LiveKit models if fallback is enabled and a local service is unavailable

### 7. Quick health checks

Run these three checks before starting the agent:

```bash
curl http://127.0.0.1:11434/api/tags
curl http://127.0.0.1:8000/v1/models
curl http://127.0.0.1:8000/v1/models
```

The second and third are intentionally the same if you use Speaches for both STT and TTS.

### 8. What to check if it still falls back to hosted models

If the agent still uses hosted models:
- confirm `USE_LOCAL_MODELS=1` is set in `.env.local`
- confirm Ollama is running with `curl http://127.0.0.1:11434/api/tags`
- confirm Speaches is running with `curl http://127.0.0.1:8000/v1/models`
- check startup logs for `local_model_health` and `model_stack_selected`

If startup fails entirely:
- keep `LOCAL_MODEL_FALLBACK_TO_LIVEKIT=1` while bringing up local services
- verify all `LOCAL_*_BASE_URL` and `LOCAL_*_HEALTH_URL` values
- verify the exact model IDs exposed by Speaches from `GET /v1/models` and update `.env.local` if needed

## Run

Run in console mode:

```bash
uv run python src/agent.py console
```

Run for frontend or telephony development:

```bash
uv run python src/agent.py dev
```

Run the production worker:

```bash
uv run python src/agent.py start
```

## Testing

Software-first unit suite:

```bash
uv run python -m pytest tests/test_tour_domain.py tests/test_engagement_service.py tests/test_navigation_service.py tests/test_session_runtime.py tests/test_knowledge_service.py tests/test_model_config.py -v
```

Existing ROS2 mocked tests:

```bash
uv run python -m pytest tests/test_ros2_connector.py -v
```

Existing ROS2 real integration tests:

```bash
source /opt/ros/humble/setup.bash && PYTEST_ADDOPTS="-p no:launch_ros -p no:launch_testing" PYTHONPATH=src:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages uv run pytest tests/test_ros2_connector_level2.py -v
```

LiveKit evals:

```bash
uv run python -m pytest tests/test_agent.py -v
```

## Current Behaviour
- Starts as a concierge-style assistant.
- Performs startup health inspection and emits a structured startup health report.
- Uses software wake phrase gating by default and supports manual always-awake mode.
- Falls back to stationary concierge mode when navigation is unavailable.
- Can select between hosted LiveKit inference and a local STT-LLM-TTS stack.
- Probes local model services before startup and can fall back to hosted inference if enabled.
- Captures lightweight session memory for explicit user names and high-level preference.
- Can hand off into tour mode.
- Uses deterministic tour state and retry or skip logic.
- Uses YAML content files plus `knowledge.py` prompt assembly.
- Keeps ROS2 connector compatibility for existing tests and scripts.

## Current Verification
- `uv run python -m compileall src`
- `uv run python -m pytest tests/test_session_runtime.py tests/test_model_config.py -v`
- previously verified: `uv run python -m pytest tests/test_tour_domain.py tests/test_engagement_service.py tests/test_navigation_service.py tests/test_session_runtime.py tests/test_knowledge_service.py -v`
- previously verified: `uv run python -m pytest tests/test_ros2_connector.py -v`

## Known Limitations
- Ollama covers only the LLM; STT and TTS still require separate local services.
- Tour stop coordinates are still placeholders until real robot validation is completed.
- Recovery and replan narration depends on what Nav2 feedback exposes on the robot.
- Wake behaviour is phrase-based, not a dedicated hotword engine.
- The multi-agent setup is intentionally minimal for now: concierge plus tour agent.
- `tests/test_agent.py` has been updated, but full execution is still pending a clean rerun with a working inference path.

## Notes For Future Developers
- If you change the runtime architecture, preserve `src/agent.py` unless you also update taskfile, Dockerfile, tests, and docs.
- If you change content formats, update `KnowledgeService` and `SessionRuntime` content loading together.
- If you change the model-provider contract, update `runtime/model_config.py`, `runtime/session_runtime.py`, `.env.example`, and this README together.
- If you change the ROS adapter surface, rerun `tests/test_ros2_connector.py` before touching higher-level logic.
- Update `PRD`, `PROGRESS.md`, `TODO.md`, and this README whenever the architecture or backlog meaningfully changes.
