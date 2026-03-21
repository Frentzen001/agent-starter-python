"""
ros2_connector.py
=================
Async-friendly ROS 2 facade for LiveKit Agents.

Design goals
------------
- Single owner of the rclpy lifecycle (init, node, executor, spin thread)
- Thread-safe publish via a drain timer on the executor's own thread
  (avoids the rcl mutex deadlock when publish() is called from asyncio)
- Thread-safe subscribe via asyncio.Queue + loop.call_soon_threadsafe()
- No side effects at import time — call start() explicitly
- Optional: safe to construct/import even when rclpy is not installed

Usage
-----
    import asyncio
    from std_msgs.msg import Int32
    from ros2_connector import ROS2Connector, ROS2_AVAILABLE

    connector = ROS2Connector()

    async def main():
        loop = asyncio.get_event_loop()
        connector.start(loop)

        # Publish
        connector.publish("/eye_expression", Int32, {"data": 1})

        # Subscribe
        queue = connector.subscribe("/chatter", String)
        msg = await queue.get()
        print(msg.data)

        connector.shutdown()
"""

from __future__ import annotations

import asyncio
import logging
import math
import queue
import threading
from typing import TYPE_CHECKING, Any

logger = logging.getLogger("ros2_connector")

# ---------------------------------------------------------------------------
# Optional ROS 2 import guard
# ---------------------------------------------------------------------------
try:
    import rclpy
    import rclpy.executors
    from rclpy.node import Node

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    logger.warning("rclpy not found — ROS2Connector will run in no-op mode.")

# ---------------------------------------------------------------------------
# Optional Nav2 import guard
# ---------------------------------------------------------------------------
try:
    from geometry_msgs.msg import PoseStamped
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

    NAV2_AVAILABLE = True
except ImportError:
    NAV2_AVAILABLE = False
    logger.warning("nav2_simple_commander not found — navigation disabled.")

if TYPE_CHECKING:
    # Only used for type hints; never imported at runtime when ROS2 is absent
    from rclpy.node import Node as NodeType  # noqa: F811


# ---------------------------------------------------------------------------
# ROS2Connector
# ---------------------------------------------------------------------------
class ROS2Connector:
    """
    Single-node, single-executor ROS 2 connector for LiveKit Agents.

    Thread model
    ------------
    - The spin thread calls rclpy.executors.MultiThreadedExecutor.spin()
      forever (daemon, so it dies with the process).
    - A 10 ms timer callback (_drain_publish_queue) runs on that thread and
      is the ONLY place that calls publisher.publish().  This eliminates the
      rcl mutex contention / hang that occurs when publish() is called from
      an asyncio coroutine / different thread.
    - Subscriber callbacks use loop.call_soon_threadsafe() to push messages
      into asyncio.Queues that callers can await in coroutines.
    """

    _DRAIN_INTERVAL_SEC: float = 0.01  # 10 ms → ~100 Hz drain

    def __init__(self) -> None:
        self._node: Any = None  # rclpy Node
        self._executor: Any = None  # rclpy MultiThreadedExecutor
        self._spin_thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None

        # topic → rclpy Publisher
        self._publishers: dict[str, Any] = {}
        # Pending publish jobs: (topic, msg_type, data_dict)
        self._publish_queue: queue.Queue[tuple[str, Any, dict[str, Any]]] = queue.Queue()

        # topic → asyncio.Queue for incoming messages
        self._sub_queues: dict[str, asyncio.Queue] = {}

        # Nav2 state
        self._navigator: Any = None  # BasicNavigator instance
        self._nav_active: bool = False
        self._cancel_requested: bool = False
        self._distance_remaining: float | None = None

        self._started = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start(self, loop: asyncio.AbstractEventLoop) -> None:
        """
        Initialise rclpy, create the node + executor, and start the spin thread.

        Parameters
        ----------
        loop:
            The running asyncio event loop.  Subscriber callbacks will use
            loop.call_soon_threadsafe() to deliver messages into asyncio.Queues.
        """
        if not ROS2_AVAILABLE:
            logger.warning("ROS2Connector.start(): rclpy not available, running as no-op.")
            return
        if self._started:
            logger.warning("ROS2Connector.start(): already started, ignoring.")
            return

        self._loop = loop

        if not rclpy.ok():
            rclpy.init()

        self._node = Node("livekit_connector")

        # Register the drain timer on the node BEFORE starting the executor
        # so there is no race between spin() and add_node()
        self._node.create_timer(self._DRAIN_INTERVAL_SEC, self._drain_publish_queue)

        self._executor = rclpy.executors.MultiThreadedExecutor()
        self._executor.add_node(self._node)

        self._spin_thread = threading.Thread(
            target=self._executor.spin,
            daemon=True,
            name="ros2_connector_spin",
        )
        self._spin_thread.start()
        self._started = True
        logger.info("ROS2Connector started (node=livekit_connector).")

        # Initialise Nav2 if available — shares the same executor/spin thread
        if NAV2_AVAILABLE:
            try:
                self._init_nav2()
            except Exception as exc:  # noqa: BLE001
                logger.warning(f"ROS2Connector: Nav2 init failed: {exc}")

    def shutdown(self) -> None:
        """Stop the executor and destroy all nodes."""
        if not self._started:
            return
        try:
            if self._executor:
                try:
                    self._executor.shutdown(wait=False)  # rclpy >= Jazzy
                except TypeError:
                    self._executor.shutdown()  # Humble: no 'wait' kwarg
            if self._navigator:
                self._navigator.destroy_node()
            if self._node:
                self._node.destroy_node()
        except Exception as exc:  # noqa: BLE001
            logger.warning(f"ROS2Connector shutdown error: {exc}")
        finally:
            self._started = False
            logger.info("ROS2Connector shut down.")

    # ------------------------------------------------------------------
    # Nav2
    # ------------------------------------------------------------------

    def _init_nav2(self) -> None:
        """Instantiate BasicNavigator and wait until Nav2 is fully active."""
        self._navigator = BasicNavigator()
        logger.info("ROS2Connector: waiting for Nav2 to become active …")
        try:
            self._navigator.waitUntilNav2Active()
            logger.info("ROS2Connector: Nav2 is active — navigator ready.")
        except Exception as exc:  # noqa: BLE001
            logger.warning(f"ROS2Connector: waitUntilNav2Active() failed: {exc}")
            try:
                self._navigator.destroy_node()
            except Exception:  # noqa: BLE001
                pass
            self._navigator = None

    def _make_pose(self, x: float, y: float, ow: float) -> Any:
        """Build a PoseStamped in the map frame (pure yaw rotation)."""
        oz = math.sqrt(max(0.0, 1.0 - ow ** 2))
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

    def _go_to_pose_sync(self, x: float, y: float, ow: float) -> str:
        """
        Blocking Nav2 navigation — call only via asyncio.to_thread().
        Returns 'succeeded', 'canceled', or 'failed'.
        """
        pose = self._make_pose(x, y, ow)

        self._nav_active = True
        self._cancel_requested = False
        self._distance_remaining = None

        try:
            self._navigator.goToPose(pose)
            logger.info("ROS2Connector: goToPose() returned, entering poll loop.")

            iteration = 0
            while True:
                is_complete = self._navigator.isTaskComplete()
                logger.info(
                    f"ROS2Connector: poll iteration={iteration} isTaskComplete={is_complete}"
                )
                if is_complete:
                    logger.info("ROS2Connector: isTaskComplete() returned True — exiting loop.")
                    break
                iteration += 1

                if self._cancel_requested:
                    self._navigator.cancelTask()
                    return "canceled"

                feedback = self._navigator.getFeedback()
                if feedback is not None:
                    try:
                        self._distance_remaining = feedback.distance_remaining
                        logger.info(
                            f"Nav2 distance_remaining={self._distance_remaining:.2f} m"
                        )
                        # Temporary fix: isTaskComplete() misses the result callback
                        # when Nav2 completes. Treat < 0.5 m as close enough to succeed.
                        if self._distance_remaining < 0.5:
                            logger.info(
                                "ROS2Connector: distance < 0.5 m — treating as succeeded."
                            )
                            return "succeeded"
                    except AttributeError:
                        logger.warning("Nav2 feedback has no distance_remaining attribute.")

        finally:
            self._nav_active = False
            self._distance_remaining = None

        result = self._navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            logger.info("ROS2Connector: goal succeeded.")
            return "succeeded"
        elif result == TaskResult.CANCELED:
            logger.info("ROS2Connector: goal canceled.")
            return "canceled"
        else:
            logger.warning("ROS2Connector: goal failed.")
            return "failed"

    def _cancel_task_sync(self) -> None:
        """Signal the navigation poll loop to cancel. Call via asyncio.to_thread()."""
        if self._nav_active:
            self._cancel_requested = True
            logger.info("ROS2Connector: cancel requested.")
        else:
            logger.info("ROS2Connector: no active navigation task to cancel.")

    async def navigate_to_pose(self, x: float, y: float, ow: float) -> str:
        """
        Async Nav2 navigation.  Non-blocking to the asyncio event loop.

        Parameters
        ----------
        x, y:
            Target position in the map frame (metres).
        ow:
            Quaternion w component for the desired heading.

        Returns
        -------
        str
            ``'succeeded'``, ``'canceled'``, or ``'failed'``.
            Returns ``'Navigation unavailable (Nav2 not loaded).'`` when Nav2
            is not present.
        """
        if self._navigator is None:
            return "Navigation unavailable (Nav2 not loaded)."
        return await asyncio.to_thread(self._go_to_pose_sync, x, y, ow)

    async def cancel_navigation(self) -> None:
        """Signal any active Nav2 goal to cancel. Safe to call at any time."""
        if not self._started:
            return
        await asyncio.to_thread(self._cancel_task_sync)

    @property
    def is_navigating(self) -> bool:
        """True while a Nav2 goal is in progress."""
        return self._nav_active

    @property
    def distance_remaining(self) -> float | None:
        """Most recent distance-to-goal feedback from Nav2 (metres), or None."""
        return self._distance_remaining

    # ------------------------------------------------------------------
    # Publish
    # ------------------------------------------------------------------
    def publish(self, topic: str, msg_type: Any, data: dict[str, Any]) -> None:
        """
        Thread-safe publish.  Can be called from any thread, including asyncio
        coroutines.

        The message is enqueued and delivered by the drain timer running on
        the executor's own thread, avoiding rcl mutex contention.

        Parameters
        ----------
        topic:
            ROS 2 topic name, e.g. ``"/eye_expression"``.
        msg_type:
            The ROS 2 message class, e.g. ``std_msgs.msg.Int32``.
        data:
            Dict mapping field names to values,
            e.g. ``{"data": 1}``.
        """
        if not ROS2_AVAILABLE or not self._started:
            return
        self._publish_queue.put_nowait((topic, msg_type, data))

    def _drain_publish_queue(self) -> None:
        """
        Timer callback — runs on the executor thread.
        Drains all pending publish jobs and calls publisher.publish() here,
        where it is safe to do so without rcl mutex contention.
        """
        while not self._publish_queue.empty():
            try:
                topic, msg_type, data = self._publish_queue.get_nowait()
            except queue.Empty:
                break

            # Lazily create publisher if this is the first publish to this topic
            if topic not in self._publishers:
                self._publishers[topic] = self._node.create_publisher(
                    msg_type, topic, 10
                )
                logger.debug(f"ROS2Connector: created publisher for {topic}")

            msg = msg_type()
            for field_name, value in data.items():
                setattr(msg, field_name, value)

            self._publishers[topic].publish(msg)
            logger.debug(f"ROS2Connector: published to {topic} data={data}")

    # ------------------------------------------------------------------
    # Subscribe
    # ------------------------------------------------------------------
    def subscribe(self, topic: str, msg_type: Any) -> asyncio.Queue:
        """
        Subscribe to a ROS 2 topic and return an asyncio.Queue.

        Incoming messages are pushed into the queue via
        loop.call_soon_threadsafe() so they can be awaited safely in
        coroutines.  Calling subscribe() again with the same topic returns
        the same queue (idempotent).

        Parameters
        ----------
        topic:
            ROS 2 topic name, e.g. ``"/scan"``.
        msg_type:
            The ROS 2 message class, e.g. ``sensor_msgs.msg.LaserScan``.

        Returns
        -------
        asyncio.Queue
            Await ``queue.get()`` in a coroutine to receive the next message.
        """
        if topic in self._sub_queues:
            return self._sub_queues[topic]

        q: asyncio.Queue = asyncio.Queue(maxsize=50)
        self._sub_queues[topic] = q

        if not ROS2_AVAILABLE or not self._started:
            # Return an empty queue as a safe no-op
            logger.warning(
                f"ROS2Connector.subscribe({topic}): connector not started, "
                "returning empty queue."
            )
            return q

        def _callback(msg: Any) -> None:
            if self._loop and not self._loop.is_closed():
                self._loop.call_soon_threadsafe(
                    lambda: q.put_nowait(msg) if not q.full() else None
                )

        self._node.create_subscription(msg_type, topic, _callback, 10)
        logger.info(f"ROS2Connector: subscribed to {topic}")
        return q

    # ------------------------------------------------------------------
    # Dunder helpers
    # ------------------------------------------------------------------
    def __repr__(self) -> str:
        state = "started" if self._started else "stopped"
        pubs = list(self._publishers.keys())
        subs = list(self._sub_queues.keys())
        return f"ROS2Connector({state}, publishers={pubs}, subscribers={subs})"
