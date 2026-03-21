<a href="https://livekit.io/">
  <img src="./.github/assets/livekit-mark.png" alt="LiveKit logo" width="100" height="100">
</a>

# LiveKit Agents Starter - Python

A complete starter project for building voice AI apps with [LiveKit Agents for Python](https://github.com/livekit/agents) and [LiveKit Cloud](https://cloud.livekit.io/).

The starter project includes:

- A simple voice AI assistant, ready for extension and customization
- A voice AI pipeline with [models](https://docs.livekit.io/agents/models) from OpenAI, Cartesia, and Deepgram served through LiveKit Cloud
  - Easily integrate your preferred [LLM](https://docs.livekit.io/agents/models/llm/), [STT](https://docs.livekit.io/agents/models/stt/), and [TTS](https://docs.livekit.io/agents/models/tts/) instead, or swap to a realtime model like the [OpenAI Realtime API](https://docs.livekit.io/agents/models/realtime/openai)
- Eval suite based on the LiveKit Agents [testing & evaluation framework](https://docs.livekit.io/agents/build/testing/)
- [LiveKit Turn Detector](https://docs.livekit.io/agents/build/turns/turn-detector/) for contextually-aware speaker detection, with multilingual support
- [Background voice cancellation](https://docs.livekit.io/home/cloud/noise-cancellation/)
- Integrated [metrics and logging](https://docs.livekit.io/agents/build/metrics/)
- A Dockerfile ready for [production deployment](https://docs.livekit.io/agents/ops/deployment/)

This starter app is compatible with any [custom web/mobile frontend](https://docs.livekit.io/agents/start/frontend/) or [SIP-based telephony](https://docs.livekit.io/agents/start/telephony/).

## Coding agents and MCP

This project is designed to work with coding agents like [Cursor](https://www.cursor.com/) and [Claude Code](https://www.anthropic.com/claude-code). 

To get the most out of these tools, install the [LiveKit Docs MCP server](https://docs.livekit.io/mcp).

For Cursor, use this link:

[![Install MCP Server](https://cursor.com/deeplink/mcp-install-light.svg)](https://cursor.com/en-US/install-mcp?name=livekit-docs&config=eyJ1cmwiOiJodHRwczovL2RvY3MubGl2ZWtpdC5pby9tY3AifQ%3D%3D)

For Claude Code, run this command:

```
claude mcp add --transport http livekit-docs https://docs.livekit.io/mcp
```

For Codex CLI, use this command to install the server:
```
codex mcp add --url https://docs.livekit.io/mcp livekit-docs
```

For Gemini CLI, use this command to install the server:
```
gemini mcp add --transport http livekit-docs https://docs.livekit.io/mcp
```

The project includes a complete [AGENTS.md](AGENTS.md) file for these assistants. You can modify this file  your needs. To learn more about this file, see [https://agents.md](https://agents.md).

## Dev Setup

Clone the repository and install dependencies to a virtual environment:

```console
cd agent-starter-python
uv sync
```

Sign up for [LiveKit Cloud](https://cloud.livekit.io/) then set up the environment by copying `.env.example` to `.env.local` and filling in the required keys:

- `LIVEKIT_URL`
- `LIVEKIT_API_KEY`
- `LIVEKIT_API_SECRET`

You can load the LiveKit environment automatically using the [LiveKit CLI](https://docs.livekit.io/home/cli/cli-setup):

```bash
lk cloud auth
lk app env -w -d .env.local
```

## Run the agent

Before your first run, you must download certain models such as [Silero VAD](https://docs.livekit.io/agents/build/turns/vad/) and the [LiveKit turn detector](https://docs.livekit.io/agents/build/turns/turn-detector/):

```console
uv run python src/agent.py download-files
```

Next, run this command to speak to your agent directly in your terminal:

```console
uv run python src/agent.py console
```

To run the agent for use with a frontend or telephony, use the `dev` command:

```console
uv run python src/agent.py dev
```

In production, use the `start` command:

```console
uv run python src/agent.py start
```

## More-Tea: Garage@EEE Tour Guide Customisations

This project extends the base LiveKit starter with two features that turn the agent into **More-Tea**, a voice-controlled robot tour guide for [Garage@EEE](https://garage.eee.ntu.edu.sg/) at NTU Open House.

### ROS 2 Eye Expression Publishing (`src/agent_example.py`)

More-Tea can display emotions on a physical robot's LED eyes by publishing integer values to the `/eye_expression` ROS 2 topic.

**How it works:**

- At startup, the agent tries to import `rclpy`. If ROS 2 is not installed, it falls back gracefully and eye expressions are disabled (no crash).
- A `ROS2EyePublisher` node is spun inside a daemon `MultiThreadedExecutor` thread so it never blocks the LiveKit event loop.
- The `set_eye_expression` function tool is exposed to the LLM. The LLM is instructed to call it at the start of every response and whenever its emotional tone changes.

**Emotion → integer mapping:**

| Emotion   | Value |
|-----------|-------|
| neutral   | 0     |
| happy     | 1     |
| sad       | 2     |
| angry     | 3     |
| confused  | 4     |
| shocked   | 5     |
| love      | 6     |
| shy       | 7     |

**ROS 2 dependency (optional):**

```bash
# Inside a sourced ROS 2 Humble workspace:
pip install rclpy
```

If `rclpy` is not available, the agent runs normally — eye expressions are simply skipped.

---

### Garage@EEE Knowledge Base (`src/knowledge.py`)

Instead of RAG, all Garage@EEE information is loaded **once at startup** from an xlsx database and injected directly into the agent's system prompt. This gives zero per-query latency and 100% recall — critical for a low-latency voice agent.

**Source file:**

```
src/Garage@EEEWebsiteDatabase.xlsx   ← place alongside knowledge.py in src/
```

The path is resolved automatically. Override it with the `GARAGE_XLSX` environment variable if needed:

```bash
export GARAGE_XLSX=/path/to/your/database.xlsx
```

**Extracted content (17 KB, ~4,400 tokens):**

| Sheet | What is extracted |
|---|---|
| Home | About text and objectives |
| Facilities | All 9 facilities with descriptions |
| Events | 7 events (STARTathon, GEEEnius, Enitio, Escendo, IdeasJam, Innovation Festival, Makerspace Friendlies) |
| Ambassadors | All 6 ambassador portfolio descriptions |
| Tinkering Project | Intro, how to join, funding ($200), FAQs |
| Innotrack | Description, funding ($999), FAQs |
| Launchpad | Description, funding ($999), FAQs |
| Project Openings | All current active projects |
| Project Info | Notable past projects |

**Dependency:**

`openpyxl` is required to parse the xlsx. It is already added to the project via `uv add openpyxl`. If the file is missing at runtime, `knowledge.py` falls back to a hardcoded minimal summary so the agent still starts.

**Running with the knowledge base:**

```console
uv run python src/agent_example.py console
```

No extra steps needed — `knowledge.py` is imported at startup and the knowledge is embedded in the system prompt automatically.

---

## Frontend & Telephony

Get started quickly with our pre-built frontend starter apps, or add telephony support:

| Platform | Link | Description |
|----------|----------|-------------|
| **Web** | [`livekit-examples/agent-starter-react`](https://github.com/livekit-examples/agent-starter-react) | Web voice AI assistant with React & Next.js |
| **iOS/macOS** | [`livekit-examples/agent-starter-swift`](https://github.com/livekit-examples/agent-starter-swift) | Native iOS, macOS, and visionOS voice AI assistant |
| **Flutter** | [`livekit-examples/agent-starter-flutter`](https://github.com/livekit-examples/agent-starter-flutter) | Cross-platform voice AI assistant app |
| **React Native** | [`livekit-examples/voice-assistant-react-native`](https://github.com/livekit-examples/voice-assistant-react-native) | Native mobile app with React Native & Expo |
| **Android** | [`livekit-examples/agent-starter-android`](https://github.com/livekit-examples/agent-starter-android) | Native Android app with Kotlin & Jetpack Compose |
| **Web Embed** | [`livekit-examples/agent-starter-embed`](https://github.com/livekit-examples/agent-starter-embed) | Voice AI widget for any website |
| **Telephony** | [📚 Documentation](https://docs.livekit.io/agents/start/telephony/) | Add inbound or outbound calling to your agent |

For advanced customization, see the [complete frontend guide](https://docs.livekit.io/agents/start/frontend/).

## Testing

This project has two test levels for the ROS 2 connector, plus the LiveKit agent evals.

---

### Level 1 — Mocked unit tests (no ROS 2 needed)

Runs on any machine. All rclpy/Nav2 imports are patched out with `unittest.mock`.

```bash
PYTHONPATH=src uv run pytest tests/test_ros2_connector.py -v
```

**What is verified:**
- `publish()` is a no-op and never raises when ROS 2 is unavailable
- `publish()` enqueues exactly one item per call
- `_drain_publish_queue()` creates a publisher lazily and calls `publisher.publish()` with the correct message
- `subscribe()` returns an `asyncio.Queue` and is idempotent (same topic → same queue)
- `shutdown()` is safe to call multiple times without errors
- `set_eye_expression` emotion → integer mapping, topic name, data dict format
- Nav2 `navigate_to_pose()` returns the "unavailable" string when `_navigator is None`
- `cancel_navigation()` sets `_cancel_requested` correctly
- `is_navigating` and `distance_remaining` properties reflect internal state

---

### Level 2 — Real rclpy integration tests (ROS 2 must be sourced)

Uses a real `rclpy` node — no mocking. Verifies the actual thread-safe bridge path
from `publish()` → drain timer → DDS → subscriber callback → `asyncio.Queue`.

**Prerequisites:**
```bash
source /opt/ros/humble/setup.bash   # or your ROS 2 distro
```

**Run:**
```bash
source /opt/ros/humble/setup.bash && \
PYTEST_ADDOPTS="-p no:launch_ros -p no:launch_testing" \
PYTHONPATH=src:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages \
uv run pytest tests/test_ros2_connector_level2.py -v
```

> **Why `PYTEST_ADDOPTS`?**
> ROS 2 Humble ships its own pytest plugins (`launch_testing`, `launch_ros`) that are
> registered as setuptools entry points. One of them (`launch_testing_ros`) depends on
> `lark` and crashes pytest during collection. Setting `PYTEST_ADDOPTS` before pytest
> starts suppresses them before entry points are loaded — `pyproject.toml`'s `addopts`
> is processed too late.

> **Why the explicit `PYTHONPATH`?**
> `uv run` creates an isolated venv that does not inherit the `PYTHONPATH` set by
> `setup.bash`. You must append the ROS 2 site-packages explicitly so that `rclpy` and
> `std_msgs` are importable inside the venv's Python.

**What is verified (with real rclpy):**
- The drain timer actually fires and calls `publisher.publish()` on a real `rclpy` publisher
- 10 messages queued → all drained in under 300 ms, no hang
- Different topics create distinct `rclpy` publisher objects
- Full pub/sub loopback: `Int32` published with `{"data": 42}` arrives in the `asyncio.Queue` with `msg.data == 42`
- 3 sequential messages arrive in publish order
- `subscribe()` idempotency holds on a live node
- `repr()`, `_started` flag, and `shutdown()` behave correctly with a real `rclpy` context

---

### Level 3 — Live robot smoke test (Nav2 + real hardware)

No automated test exists for this level. Run manually after deploying to the robot:

```bash
# Terminal 1 — start the agent
uv run src/agent.py dev

# Terminal 2 — watch eye expression topic
source /opt/ros/humble/setup.bash
ros2 topic echo /eye_expression std_msgs/msg/Int32

# Terminal 3 — watch nav2 (if Nav2 is running)
ros2 topic echo /navigate_to_pose/status action_msgs/msg/GoalStatusArray
```

Say "start the tour" to the agent and verify the robot starts moving.

---

### Agent behaviour evals (LiveKit)

Tests the LLM-facing behaviour of the agent (grounding, friendliness, tool use):

```bash
PYTHONPATH=src uv run pytest tests/test_agent.py -v
```

Requires `LIVEKIT_URL`, `LIVEKIT_API_KEY`, and `LIVEKIT_API_SECRET` in `.env.local`.

---

### Quick reference

| Command | What it tests | Needs ROS 2? |
|---|---|---|
| `PYTHONPATH=src uv run pytest tests/test_ros2_connector.py -v` | ROS 2 connector logic (mocked) | No |
| *(see Level 2 command above)* | Real rclpy pub/sub bridge | Yes (sourced) |
| `PYTHONPATH=src uv run pytest tests/test_agent.py -v` | LiveKit agent evals | No (uses LiveKit Cloud) |
| `PYTHONPATH=src uv run pytest -v` | All of the above | No |

## Using this template repo for your own project

Once you've started your own project based on this repo, you should:

1. **Check in your `uv.lock`**: This file is currently untracked for the template, but you should commit it to your repository for reproducible builds and proper configuration management. (The same applies to `livekit.toml`, if you run your agents in LiveKit Cloud)

2. **Remove the git tracking test**: Delete the "Check files not tracked in git" step from `.github/workflows/tests.yml` since you'll now want this file to be tracked. These are just there for development purposes in the template repo itself.

3. **Add your own repository secrets**: You must [add secrets](https://docs.github.com/en/actions/how-tos/writing-workflows/choosing-what-your-workflow-does/using-secrets-in-github-actions) for `LIVEKIT_URL`, `LIVEKIT_API_KEY`, and `LIVEKIT_API_SECRET` so that the tests can run in CI.

## Deploying to production

This project is production-ready and includes a working `Dockerfile`. To deploy it to LiveKit Cloud or another environment, see the [deploying to production](https://docs.livekit.io/agents/ops/deployment/) guide.

## Self-hosted LiveKit

You can also self-host LiveKit instead of using LiveKit Cloud. See the [self-hosting](https://docs.livekit.io/home/self-hosting/) guide for more information. If you choose to self-host, you'll need to also use [model plugins](https://docs.livekit.io/agents/models/#plugins) instead of LiveKit Inference and will need to remove the [LiveKit Cloud noise cancellation](https://docs.livekit.io/home/cloud/noise-cancellation/) plugin.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
