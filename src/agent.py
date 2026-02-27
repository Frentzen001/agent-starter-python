"""
Voice-Controlled Robotic Tour Guide for Garage@EEE
====================================================
LiveKit Agent  x  Nav2 (BasicNavigator)  x  ROS 2 Humble

Architecture
------------
  ROS 2 nodes and Nav2 blocking calls run inside a MultiThreadedExecutor
  that is spun in a dedicated daemon thread.  All long-running Nav2
  operations are dispatched with asyncio.to_thread() so the LiveKit
  event loop never blocks.

Coordinate frame
----------------
  All (x, y) positions are expressed in the *map* frame.
  The orientation quaternion (ox, oy, oz, ow) encodes the robot's
  desired heading on arrival.  Update TOUR_MANIFEST to match your
  actual map after running SLAM / AMCL.
"""

import asyncio
import logging
import re
import threading
import time
from dataclasses import dataclass, field
from typing import NamedTuple

from dotenv import load_dotenv
from playwright.async_api import async_playwright

# ---------------------------------------------------------------------------
# ROS 2 optional imports — split so eye publisher works without Nav2
# ---------------------------------------------------------------------------

# Core ROS 2 (rclpy + std_msgs) — needed for eye expression publishing
try:
    import rclpy
    import rclpy.executors
    from rclpy.node import Node
    from std_msgs.msg import Int32

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    _log_tmp = logging.getLogger("agent")
    _log_tmp.warning("rclpy not found — ROS 2 eye expression publishing disabled.")

# Nav2 (nav2_simple_commander + geometry_msgs) — needed for navigation
try:
    from geometry_msgs.msg import PoseStamped
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

    NAV2_AVAILABLE = True
except ImportError:
    NAV2_AVAILABLE = False
    _log_tmp2 = logging.getLogger("agent")
    _log_tmp2.warning(
        "nav2_simple_commander not found — robot navigation disabled. "
        "Install it inside a sourced ROS 2 Humble workspace."
    )

from livekit import rtc
from livekit.agents import (
    Agent,
    AgentServer,
    AgentSession,
    JobContext,
    JobProcess,
    RunContext,
    cli,
    function_tool,
    inference,
    room_io,
)
from livekit.plugins import noise_cancellation, silero
from livekit.plugins.turn_detector.multilingual import MultilingualModel

logger = logging.getLogger("agent")
load_dotenv(".env.local")

# ---------------------------------------------------------------------------
# Eye expression mapping
# ---------------------------------------------------------------------------
EMOTION_MAP: dict[str, int] = {
    "neutral": 0,
    "happy": 1,
    "sad": 2,
    "angry": 3,
    "confused": 4,
    "shocked": 5,
    "love": 6,
    "shy": 7,
}


# ---------------------------------------------------------------------------
# Tour Manifest  — UPDATE x/y/ow to match your real map!
# Each entry: location_name -> (x, y, ow)
#   (x, y)  — position in the map frame (metres)
#   ow      — quaternion w component for heading (simplified yaw=0 → ow=1.0)
# ---------------------------------------------------------------------------
class Waypoint(NamedTuple):
    x: float
    y: float
    ow: float  # quaternion w for heading (oz computed as sqrt(1-ow²) if needed)


TOUR_MANIFEST: dict[str, Waypoint] = {
    "Entrance": Waypoint(x=0.00, y=0.00, ow=1.000),
    "3D Printing Station": Waypoint(x=2.50, y=1.00, ow=0.924),
    "Electronics Lab": Waypoint(x=5.00, y=0.50, ow=0.924),
    "Laser Cutting Area": Waypoint(x=5.00, y=-2.00, ow=0.707),
    "Wood & Metal Workshop": Waypoint(x=2.50, y=-3.50, ow=0.000),
    "Collaboration Hub": Waypoint(x=0.00, y=-3.00, ow=-0.707),
    "Project Showcase Wall": Waypoint(x=-2.00, y=-1.50, ow=-0.924),
    "Exit": Waypoint(x=-2.00, y=0.00, ow=-1.000),
}

# Ordered stop list used by start_tour / resume_tour
TOUR_STOPS: list[str] = list(TOUR_MANIFEST.keys())


# ---------------------------------------------------------------------------
# Nav2Controller
# ---------------------------------------------------------------------------
class Nav2Controller:
    """
    Thin async-friendly wrapper around BasicNavigator.

    All *_sync methods are intended to be called via asyncio.to_thread()
    so the LiveKit event loop is never blocked.
    """

    def __init__(self) -> None:
        if not rclpy.ok():
            rclpy.init()

        self._navigator = BasicNavigator()
        self._executor = rclpy.executors.MultiThreadedExecutor()
        self._executor.add_node(self._navigator)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True, name="nav2_executor"
        )
        self._spin_thread.start()

        # Shared feedback state (written by nav thread, read by async tasks)
        self._distance_remaining: float | None = None
        self._nav_active: bool = False
        self._cancel_requested: bool = False

        logger.info("Nav2Controller: waiting for Nav2 to become active …")
        self._navigator.waitUntilNav2Active()
        logger.info("Nav2Controller: Nav2 is active.")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _make_pose(self, x: float, y: float, ow: float) -> "PoseStamped":
        """Build a PoseStamped in the map frame."""
        # Derive oz from ow so the quaternion is normalised (pure yaw rotation)
        import math

        oz = math.sqrt(max(0.0, 1.0 - ow**2))
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self._navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = oz
        pose.pose.orientation.w = float(ow)
        return pose

    # ------------------------------------------------------------------
    # Blocking (sync) methods — call via asyncio.to_thread()
    # ------------------------------------------------------------------
    def go_to_pose_sync(
        self,
        x: float,
        y: float,
        ow: float,
        poll_interval: float = 0.5,
    ) -> str:
        """
        Send a navigation goal and block until it completes or is cancelled.
        Returns 'succeeded', 'canceled', or 'failed'.
        """
        pose = self._make_pose(x, y, ow)
        self._navigator.goToPose(pose)
        self._nav_active = True
        self._cancel_requested = False
        self._distance_remaining = None

        try:
            while not self._navigator.isTaskComplete():
                if self._cancel_requested:
                    self._navigator.cancelTask()
                    # Wait for Nav2 to acknowledge the cancel
                    while not self._navigator.isTaskComplete():
                        time.sleep(0.1)
                    return "canceled"

                feedback = self._navigator.getFeedback()
                if feedback:
                    self._distance_remaining = feedback.distance_remaining
                    logger.debug(
                        f"Nav2 distance_remaining={self._distance_remaining:.2f} m"
                    )
                time.sleep(poll_interval)
        finally:
            self._nav_active = False
            self._distance_remaining = None

        result = self._navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            return "succeeded"
        elif result == TaskResult.CANCELED:
            return "canceled"
        else:
            return "failed"

    def cancel_task_sync(self) -> None:
        """Signal the navigation loop to cancel the current goal."""
        if self._nav_active:
            self._cancel_requested = True
            logger.info("Nav2Controller: cancel requested.")
        else:
            logger.info("Nav2Controller: no active task to cancel.")

    # ------------------------------------------------------------------
    # Properties readable from any thread
    # ------------------------------------------------------------------
    @property
    def distance_remaining(self) -> float | None:
        return self._distance_remaining

    @property
    def is_navigating(self) -> bool:
        return self._nav_active

    # ------------------------------------------------------------------
    def shutdown(self) -> None:
        try:
            self._executor.shutdown(wait=False)
            self._navigator.destroy_node()
        except Exception as exc:
            logger.warning(f"Nav2Controller shutdown error: {exc}")


# ---------------------------------------------------------------------------
# ROS 2 Eye Publisher
# ---------------------------------------------------------------------------
class ROS2EyePublisher:
    """Publishes eye-expression integers to /eye_expression."""

    def __init__(self, executor: "rclpy.executors.Executor") -> None:
        self._node = Node("more_tea_eye")
        self._pub = self._node.create_publisher(Int32, "/eye_expression", 10)
        executor.add_node(self._node)
        logger.info("ROS2EyePublisher: ready, publishing to /eye_expression")

    def publish(self, value: int) -> None:
        msg = Int32()
        msg.data = value
        self._pub.publish(msg)
        logger.info(f"ROS2: published eye_expression={value}")

    def shutdown(self) -> None:
        self._node.destroy_node()


# ---------------------------------------------------------------------------
# TourManager
# ---------------------------------------------------------------------------
@dataclass
class TourManager:
    """
    Manages tour state and dispatches Nav2 goals.

    Attributes
    ----------
    current_stop_index : int
        Index into TOUR_STOPS for the *next* destination.  Incremented
        after each successful arrival so resume_tour() always picks up
        where the tour left off.
    is_active : bool
        True while a tour sequence is running.
    """

    nav: Nav2Controller | None = None
    current_stop_index: int = 0
    is_active: bool = False
    _nav_task: asyncio.Task | None = field(default=None, repr=False)

    # ------------------------------------------------------------------
    # Internal navigation dispatcher
    # ------------------------------------------------------------------
    async def _navigate_to(self, name: str, wp: Waypoint) -> str:
        """
        Drive to *wp* asynchronously (non-blocking to the event loop).
        Returns a human-readable result string.
        """
        if self.nav is None:
            return f"Navigation unavailable (ROS 2 not loaded). Pretending to drive to {name}."

        logger.info(f"TourManager: navigating to {name} ({wp})")
        result = await asyncio.to_thread(self.nav.go_to_pose_sync, wp.x, wp.y, wp.ow)
        return result

    # ------------------------------------------------------------------
    # Public async actions (called by Agent function tools)
    # ------------------------------------------------------------------
    async def start_tour(self) -> str:
        """Begin the guided tour from the very first stop."""
        self.current_stop_index = 0
        self.is_active = True
        return await self._advance_to_next()

    async def go_to_location(self, location_name: str) -> str:
        """
        Navigate directly to a named location.

        Returns a status string the LLM can narrate.
        """
        key = self._fuzzy_match(location_name)
        if key is None:
            available = ", ".join(TOUR_MANIFEST.keys())
            return f"Unknown location '{location_name}'. Available stops: {available}."
        wp = TOUR_MANIFEST[key]
        result = await self._navigate_to(key, wp)
        if result == "succeeded":
            # Update tour index to the stop after this one
            if key in TOUR_STOPS:
                self.current_stop_index = TOUR_STOPS.index(key) + 1
            return f"Arrived at {key} successfully."
        return f"Navigation to {key} ended with status: {result}."

    async def pause_tour(self) -> str:
        """Cancel the current Nav2 goal and pause the tour."""
        self.is_active = False
        if self.nav is None:
            return "Navigation unavailable — tour paused conceptually."
        await asyncio.to_thread(self.nav.cancel_task_sync)
        return "Tour paused. The robot has stopped."

    async def resume_tour(self) -> str:
        """Resume the tour from the current stop index."""
        if self.current_stop_index >= len(TOUR_STOPS):
            self.is_active = False
            return "The tour is already complete — we visited all stops!"
        self.is_active = True
        return await self._advance_to_next()

    async def stop_robot(self) -> str:
        """
        Emergency stop — immediately cancels the active Nav2 goal.
        Should be triggered on 'Stop!' or 'Watch out!' utterances.
        """
        self.is_active = False
        if self.nav is None:
            return "Navigation unavailable — stop acknowledged."
        await asyncio.to_thread(self.nav.cancel_task_sync)
        return "Robot stopped immediately!"

    def get_distance_status(self) -> str:
        """Return a human-readable distance-remaining string for status updates."""
        if self.nav is None or not self.nav.is_navigating:
            return "The robot is not currently navigating."
        dist = self.nav.distance_remaining
        if dist is None:
            return "Navigating — distance data not yet available."
        return f"Approximately {dist:.1f} metres remaining to the destination."

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    async def _advance_to_next(self) -> str:
        if self.current_stop_index >= len(TOUR_STOPS):
            self.is_active = False
            return "We have reached the end of the tour. All stops visited!"

        name = TOUR_STOPS[self.current_stop_index]
        wp = TOUR_MANIFEST[name]
        result = await self._navigate_to(name, wp)

        if result == "succeeded":
            self.current_stop_index += 1
            remaining = len(TOUR_STOPS) - self.current_stop_index
            next_name = (
                TOUR_STOPS[self.current_stop_index]
                if self.current_stop_index < len(TOUR_STOPS)
                else None
            )
            msg = f"Arrived at {name}."
            if next_name:
                msg += f" Next stop on the tour is: {next_name}. ({remaining} stop(s) left.)"
            else:
                msg += " This was the last stop — tour complete!"
            return msg
        else:
            return f"Navigation to {name} ended with status: {result}."

    def _fuzzy_match(self, query: str) -> str | None:
        """Case-insensitive substring match against manifest keys."""
        q = query.lower().strip()
        for key in TOUR_MANIFEST:
            if q in key.lower() or key.lower() in q:
                return key
        return None


# ---------------------------------------------------------------------------
# Global ROS 2 singletons (initialised once at import time)
# ---------------------------------------------------------------------------
_nav_controller: Nav2Controller | None = None
_eye_publisher: ROS2EyePublisher | None = None
_tour_manager: TourManager = TourManager()

if NAV2_AVAILABLE:
    # Full stack: share one MultiThreadedExecutor between Nav2 and the eye node
    try:
        _nav_controller = Nav2Controller()
        _eye_publisher = ROS2EyePublisher(_nav_controller._executor)
        _tour_manager.nav = _nav_controller
        logger.info("ROS 2 + Nav2 singletons initialised successfully.")
    except Exception as exc:
        logger.warning(f"Nav2Controller init failed: {exc}")

if _eye_publisher is None and ROS2_AVAILABLE:
    # rclpy available but nav2_simple_commander missing:
    # spin a lightweight standalone executor just for the eye publisher.
    try:
        if not rclpy.ok():
            rclpy.init()
        _standalone_executor = rclpy.executors.MultiThreadedExecutor()
        _ros2_eye_thread = threading.Thread(
            target=_standalone_executor.spin,
            daemon=True,
            name="ros2_eye_executor",
        )
        _ros2_eye_thread.start()
        _eye_publisher = ROS2EyePublisher(_standalone_executor)
        logger.info(
            "ROS 2 eye publisher initialised (standalone — Nav2 not available)."
        )
    except Exception as exc:
        logger.warning(f"ROS 2 eye publisher init failed: {exc}")


# ---------------------------------------------------------------------------
# LiveKit Agent
# ---------------------------------------------------------------------------
class Assistant(Agent):
    def __init__(self) -> None:
        super().__init__(
            instructions=f"""You are a friendly, witty robot tour guide named more-tea.
You help students explore Garage@EEE, an amazing makerspace full of tools, equipment, and creativity.

TOUR STOPS (in order):
{chr(10).join(f"  {i + 1}. {name}" for i, name in enumerate(TOUR_STOPS))}

BEHAVIOUR RULES:
1. Eye expressions — call set_eye_expression at the START of every spoken response and again
   whenever your emotional tone changes.  Available: neutral, happy, sad, angry, confused,
   shocked, love, shy.

2. Navigation commands — use the provided tools (start_tour, go_to_location, pause_tour,
   resume_tour, stop_robot) to physically move the robot.  Never pretend to move; always
   call the tool.

3. Interruptions — if the user asks a question while the robot is moving, call pause_tour
   first, answer clearly and helpfully, then ask: "Shall I continue to [next stop name]?"

4. Emergency stops — if the user says "Stop!", "Watch out!", "Danger!", or anything urgent,
   IMMEDIATELY call stop_robot before doing anything else.

5. Status updates — you can call get_navigation_status at any time to check how far the
   robot still needs to travel and relay that to the user (e.g. "We are almost there — just
   2 metres to go!").

6. Garage info — for questions about facilities, equipment, events, opening hours or
   anything else Garage-related, use lookup_garage_info to fetch live data from the website.

7. Keep responses concise and conversational.  No markdown, no asterisks, no emojis in
   speech — the output goes directly to text-to-speech.

Coordinates are in the *map* frame.  You do not need to handle raw numbers; tools do that.
""",
        )

    # ------------------------------------------------------------------
    # Eye expression
    # ------------------------------------------------------------------
    @function_tool()
    async def set_eye_expression(self, context: RunContext, emotion: str) -> str:
        """Set the robot's eye display to convey an emotion.

        Call this at the start of every response and again whenever your emotional tone changes.

        Args:
            emotion: One of: neutral, happy, sad, angry, confused, shocked, love, shy.
        """
        emotion = emotion.lower().strip()
        value = EMOTION_MAP.get(emotion, 0)
        logger.info(f"Eye expression → {emotion} ({value})")
        if _eye_publisher:
            _eye_publisher.publish(value)
        return f"Eye expression set to {emotion}."

    # ------------------------------------------------------------------
    # Tour tools
    # ------------------------------------------------------------------
    @function_tool()
    async def start_tour(self, context: RunContext) -> str:
        """Begin the guided tour of Garage@EEE from the first stop (Entrance).

        Call this when the user asks to start, begin, or take a tour.
        """
        await context.session.say(
            "Fantastic! Let's start the tour! Our first stop is the Entrance. "
            "Follow me — I'll lead the way!",
            allow_interruptions=True,
        )
        return await _tour_manager.start_tour()

    @function_tool()
    async def go_to_location(self, context: RunContext, location_name: str) -> str:
        """Navigate directly to a specific named location in Garage@EEE.

        Use this when a user asks to go to a particular spot by name, skipping the
        sequential tour order.

        Args:
            location_name: The name of the destination, e.g. "Electronics Lab" or "Exit".
        """
        await context.session.say(
            f"Sure! Heading to the {location_name} right now.",
            allow_interruptions=True,
        )
        return await _tour_manager.go_to_location(location_name)

    @function_tool()
    async def pause_tour(self, context: RunContext) -> str:
        """Pause the current navigation and stop the robot in place.

        Call this when the user asks to pause, wait, or stop briefly, or when you need
        to answer a question during navigation.
        """
        result = await _tour_manager.pause_tour()
        return result

    @function_tool()
    async def resume_tour(self, context: RunContext) -> str:
        """Resume the guided tour from the last checkpoint.

        Call this when the user says they are ready to continue after a pause.
        The robot will navigate to the next scheduled stop.
        """
        if _tour_manager.current_stop_index >= len(TOUR_STOPS):
            return "The tour is already complete — we have visited all stops!"
        next_name = TOUR_STOPS[_tour_manager.current_stop_index]
        await context.session.say(
            f"Great! Continuing the tour — next stop is {next_name}!",
            allow_interruptions=True,
        )
        return await _tour_manager.resume_tour()

    @function_tool()
    async def stop_robot(self, context: RunContext) -> str:
        """Emergency stop — immediately halts all robot movement.

        Trigger this tool the instant the user says anything urgent such as
        'Stop!', 'Watch out!', 'Danger!', or 'Hold on!'.
        """
        result = await _tour_manager.stop_robot()
        logger.warning("EMERGENCY STOP triggered by user.")
        return result

    @function_tool()
    async def get_navigation_status(self, context: RunContext) -> str:
        """Query how far the robot still needs to travel to the current destination.

        Use this to give the user live progress updates such as
        'We are almost there — just 2 metres to go!'.
        """
        return _tour_manager.get_distance_status()

    # ------------------------------------------------------------------
    # Garage info lookup (web scrape)
    # ------------------------------------------------------------------
    @function_tool()
    async def lookup_garage_info(self, context: RunContext, query: str) -> str:
        """Fetch live information about Garage@EEE from its official website.

        Use this for any question about facilities, equipment, events, opening hours,
        workshops, membership, or anything else makerspace-related.

        Args:
            query: The topic or question to look up.
        """
        logger.info(f"Garage info lookup: {query!r}")
        await context.session.say(
            "Let me check the Garage website for you — just a moment!",
            allow_interruptions=False,
        )
        try:
            async with async_playwright() as p:
                browser = await p.chromium.launch(headless=True)
                page = await browser.new_page()
                await page.goto(
                    "https://garage-eee.com/",
                    wait_until="networkidle",
                    timeout=20_000,
                )
                text = await page.inner_text("body")
                text = re.sub(r"\s+", " ", text).strip()
                await browser.close()
            logger.info(f"Scraped garage-eee.com: {len(text)} chars")
            return text[:6000]
        except Exception as exc:
            logger.error(f"Garage fetch error: {exc}", exc_info=True)
            return f"Error fetching Garage info: {type(exc).__name__}: {exc}"


# ---------------------------------------------------------------------------
# LiveKit server wiring
# ---------------------------------------------------------------------------
server = AgentServer()


def prewarm(proc: JobProcess) -> None:
    proc.userdata["vad"] = silero.VAD.load()


server.setup_fnc = prewarm


@server.rtc_session(agent_name="my-agent")
async def my_agent(ctx: JobContext) -> None:
    ctx.log_context_fields = {"room": ctx.room.name}

    session = AgentSession(
        # STT — Deepgram Nova 3 with multilingual support
        stt=inference.STT(model="deepgram/nova-3", language="multi"),
        # LLM — GPT-4o for tool-calling reliability
        llm=inference.LLM(model="openai/gpt-4o"),
        # TTS — Cartesia Sonic 3
        tts=inference.TTS(
            model="cartesia/sonic-3",
            voice="9626c31c-bec5-4cca-baa8-f8ba9e84c8bc",
        ),
        turn_detection=MultilingualModel(),
        vad=ctx.proc.userdata["vad"],
        preemptive_generation=True,
    )

    await session.start(
        agent=Assistant(),
        room=ctx.room,
        room_options=room_io.RoomOptions(
            audio_input=room_io.AudioInputOptions(
                noise_cancellation=lambda params: (
                    noise_cancellation.BVCTelephony()
                    if params.participant.kind
                    == rtc.ParticipantKind.PARTICIPANT_KIND_SIP
                    else noise_cancellation.BVC()
                ),
            ),
        ),
    )

    await ctx.connect()


if __name__ == "__main__":
    cli.run_app(server)
