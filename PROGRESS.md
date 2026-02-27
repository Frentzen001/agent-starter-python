# FYP Progress Report — Voice-Controlled Robotic Tour Guide

**Project:** more-tea — Garage@EEE Robot Tour Guide  
**Stack:** LiveKit Agents (Python) · Nav2 (BasicNavigator) · ROS 2 Humble  
**Last updated:** 26 February 2026 (rev 2)

---

## Overview

The LiveKit agent (`src/agent.py`) was redesigned from a simple voice assistant into a full **Voice-Controlled Robotic Tour Guide**. The agent runs on the robot's onboard computer, accepts spoken commands over LiveKit, and drives the robot through a sequence of predefined waypoints in the Garage@EEE makerspace using Nav2.

---

## Architecture

```
LiveKit event loop (asyncio)
    │
    ├─ Assistant (Agent) ─── function tools
    │       set_eye_expression       → ROS2EyePublisher  ─┐
    │       start_tour                                     │
    │       go_to_location           → TourManager         │
    │       pause_tour                    │                │
    │       resume_tour                   ▼                │
    │       stop_robot            Nav2Controller           │
    │       get_navigation_status    asyncio.to_thread()   │
    │       lookup_garage_info     (blocks in thread)      │
    │                                     │                │
    └─────────────────────── MultiThreadedExecutor ────────┘
                              (daemon thread)
                            BasicNavigator  +  eye Node
```

All blocking ROS 2 / Nav2 calls are dispatched with `asyncio.to_thread()` so the LiveKit event loop is never stalled.

---

## Components Implemented

### 1. `Nav2Controller`
- Wraps `nav2_simple_commander.BasicNavigator` in a `rclpy.executors.MultiThreadedExecutor` spun on a dedicated daemon thread.
- `go_to_pose_sync(x, y, ow)` — sends a `PoseStamped` goal (in the `map` frame), polls `getFeedback()` every 0.5 s, stores live `distance_remaining`, and respects a `cancel_requested` flag for mid-navigation cancels.
- `cancel_task_sync()` — signals the polling loop to call `navigator.cancelTask()` and waits for Nav2 acknowledgement.
- Quaternion `oz` is derived automatically from the supplied `ow` so callers only need to specify a single heading value.

### 2. `ROS2EyePublisher`
- Publishes `std_msgs/Int32` to `/eye_expression` topic.
- Node is added to the same `MultiThreadedExecutor` as Nav2 — no extra thread, single `rclpy.init()` call.

### 3. `TourManager`
| Method | Behaviour |
|---|---|
| `start_tour()` | Resets `current_stop_index` to 0 and drives to the first stop. |
| `go_to_location(name)` | Fuzzy-matches a location name and navigates directly to it; updates the index for resume. |
| `pause_tour()` | Cancels the active Nav2 goal; sets `is_active = False`. |
| `resume_tour()` | Resumes from `current_stop_index` — always picks up from the last incomplete stop. |
| `stop_robot()` | Emergency cancel — identical to pause but logged as a warning. |
| `get_distance_status()` | Reads live `distance_remaining` from Nav2 feedback and returns a human-readable string. |

### 4. Tour Manifest (`TOUR_MANIFEST`)
Eight named waypoints in the `map` frame — **update x/y/ow after your SLAM/AMCL run**:

| # | Location | x | y | ow (heading) |
|---|---|---|---|---|
| 1 | Entrance | 0.00 | 0.00 | 1.000 |
| 2 | 3D Printing Station | 2.50 | 1.00 | 0.924 |
| 3 | Electronics Lab | 5.00 | 0.50 | 0.924 |
| 4 | Laser Cutting Area | 5.00 | −2.00 | 0.707 |
| 5 | Wood & Metal Workshop | 2.50 | −3.50 | 0.000 |
| 6 | Collaboration Hub | 0.00 | −3.00 | −0.707 |
| 7 | Project Showcase Wall | −2.00 | −1.50 | −0.924 |
| 8 | Exit | −2.00 | 0.00 | −1.000 |

### 5. Function Tools (LLM Actions)

| Tool | Trigger | Action |
|---|---|---|
| `set_eye_expression` | Start of every response / emotion change | Publishes integer to `/eye_expression` |
| `start_tour` | "Start/begin the tour" | Resets index → navigates to Stop 1 |
| `go_to_location` | "Take me to [place]" | Direct navigation to named stop |
| `pause_tour` | "Pause", "Wait", mid-tour question | Cancels Nav2 goal |
| `resume_tour` | "Continue", "Let's go" | Resumes from last checkpoint |
| `stop_robot` | "Stop!", "Watch out!", "Danger!" | Emergency Nav2 cancel |
| `get_navigation_status` | "How far?", "Are we close?" | Returns live distance remaining |
| `lookup_garage_info` | Any Garage facility/event question | Scrapes garage-eee.com |

### 6. System Prompt Behaviour Rules
- Calls `set_eye_expression` at the start of **every** response.
- On mid-tour questions → calls `pause_tour`, answers, then asks: *"Shall I continue to [next stop]?"*
- On urgent utterances ("Stop!", "Watch out!") → calls `stop_robot` **before** any other action.
- Uses `get_navigation_status` to narrate live distance (e.g. *"Just 2 metres to go!"*).
- Uses `lookup_garage_info` for all makerspace factual queries.

### 7. Voice Pipeline
| Component | Model |
|---|---|
| STT | Deepgram Nova-3 (multilingual) |
| LLM | OpenAI GPT-4o |
| TTS | Cartesia Sonic-3 |
| Turn detection | LiveKit MultilingualModel |
| VAD | Silero (prewarmed) |

---

## Files Changed

| File | Change |
|---|---|
| `src/agent.py` | Full rewrite — added `Nav2Controller`, `ROS2EyePublisher`, `TourManager`, 8 function tools, updated system prompt and voice pipeline |
| `src/agent.py` | Bug fix — split `ROS2_AVAILABLE` / `NAV2_AVAILABLE` flags so eye publisher works independently of Nav2 |
| `PROGRESS.md` | Created this report |

---

## Bug Fixes & Incremental Changes

### Rev 2 — Split ROS 2 import flags (26 Feb 2026)

**Problem:** The original code wrapped `rclpy` and `nav2_simple_commander` in a single `try` block. Because `nav2_simple_commander` is only available in a sourced ROS 2 Humble workspace (not in the plain `uv` venv), the entire block raised `ImportError`, setting `ROS2_AVAILABLE = False`. This silently disabled the eye expression publisher even though `rclpy` itself was perfectly importable.

**Confirmed startup state before fix:**
```
ROS2_AVAILABLE: False   ← wrong, rclpy IS available
_eye_publisher: None    ← broken
```

**Fix:** Separated into two independent feature flags:

| Flag | Guards | Status on dev machine |
|---|---|---|
| `ROS2_AVAILABLE` | `rclpy`, `Node`, `Int32`, `ROS2EyePublisher` | ✅ True |
| `NAV2_AVAILABLE` | `nav2_simple_commander`, `BasicNavigator`, `Nav2Controller` | ❌ False (robot only) |

The singleton init block now has two stages:
1. If `NAV2_AVAILABLE` → start `Nav2Controller` with shared executor + attach eye publisher to it.
2. Else if `ROS2_AVAILABLE` → spin a lightweight standalone `MultiThreadedExecutor` just for the eye publisher.

**Confirmed startup state after fix:**
```
ROS2_AVAILABLE: True
NAV2_AVAILABLE: False
_eye_publisher: <ROS2EyePublisher object>   ✅ live
_nav_controller: None                        (expected — robot only)
```

---

## Next Steps / TODO

- [x] Fix eye expression publisher silently disabled when `nav2_simple_commander` missing → split `ROS2_AVAILABLE` / `NAV2_AVAILABLE` flags.
- [ ] Install `nav2_simple_commander` on the robot (source ROS 2 Humble workspace before running the agent).
- [ ] Run SLAM on the actual Garage@EEE floor plan and capture real (x, y, ow) values for each stop.
- [ ] Update `TOUR_MANIFEST` in `src/agent.py` with the measured coordinates.
- [ ] Test Nav2 goal feedback loop on the physical robot.
- [ ] Write/extend unit tests in `tests/test_agent.py` covering tour state transitions.
- [ ] Add a `get_tour_status` tool so users can ask "Where are we in the tour?".
- [ ] Explore LiveKit handoff/workflow pattern to split the tour narration agent from the navigation controller agent for reduced LLM context.
- [ ] Evaluate switching TTS voice for better robot character fit.
