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

    def shutdown(self) -> None:
        """Stop the executor and destroy the node."""
        if not self._started:
            return
        try:
            if self._executor:
                self._executor.shutdown(wait=False)
            if self._node:
                self._node.destroy_node()
        except Exception as exc:  # noqa: BLE001
            logger.warning(f"ROS2Connector shutdown error: {exc}")
        finally:
            self._started = False
            logger.info("ROS2Connector shut down.")

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
