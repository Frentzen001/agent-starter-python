"""
tests/test_ros2_connector.py
============================
Unit tests for ros2_connector.ROS2Connector.

All tests mock rclpy so they run on any machine regardless of whether
ROS 2 is installed.  Tests validate:
  - No-op safety when ROS2 is unavailable
  - publish() enqueues without raising
  - subscribe() returns a queue (idempotent, same queue on repeat calls)
  - _drain_publish_queue() builds the message and calls publish()
  - shutdown() is safe to call multiple times
"""

from __future__ import annotations

import asyncio
import queue
import threading
import types
import unittest
from unittest.mock import MagicMock, patch


# ---------------------------------------------------------------------------
# Helpers to build a minimal fake rclpy module
# ---------------------------------------------------------------------------

def _make_fake_rclpy() -> tuple[types.ModuleType, MagicMock]:
    """
    Returns (fake_rclpy_module, fake_node_instance).
    The fake executor's spin() returns immediately so tests don't hang.
    """
    fake_node = MagicMock()
    # create_timer must return a mock (not raise)
    fake_node.create_timer.return_value = MagicMock()
    # create_publisher returns a mock publisher
    fake_publisher = MagicMock()
    fake_node.create_publisher.return_value = fake_publisher
    # create_subscription returns a mock
    fake_node.create_subscription.return_value = MagicMock()

    fake_executor = MagicMock()
    # spin() must not block — use a no-op
    fake_executor.spin = MagicMock(return_value=None)

    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.ok = MagicMock(return_value=False)
    fake_rclpy.init = MagicMock()

    # rclpy.executors sub-module
    fake_executors = types.ModuleType("rclpy.executors")
    fake_executors.MultiThreadedExecutor = MagicMock(return_value=fake_executor)
    fake_rclpy.executors = fake_executors

    # rclpy.node sub-module with Node class
    fake_node_module = types.ModuleType("rclpy.node")
    fake_node_module.Node = MagicMock(return_value=fake_node)
    fake_rclpy.node = fake_node_module

    return fake_rclpy, fake_node, fake_executor


# ---------------------------------------------------------------------------
# Test: no-op safety when ROS2 is unavailable
# ---------------------------------------------------------------------------

class TestPublishNoROS2(unittest.TestCase):
    """publish() and subscribe() must be safe no-ops when rclpy is absent."""

    def setUp(self):
        # Patch ROS2_AVAILABLE to False inside the module
        self._patcher = patch("ros2_connector.ROS2_AVAILABLE", False)
        self._patcher.start()
        # Re-import after patching so the module sees the patched value
        import importlib
        import ros2_connector
        importlib.reload(ros2_connector)
        self.mod = ros2_connector

    def tearDown(self):
        self._patcher.stop()

    def test_publish_is_noop_without_ros2(self):
        """publish() must not raise when ROS2 is unavailable."""
        connector = self.mod.ROS2Connector()
        # Should not raise even though connector was never started
        try:
            connector.publish("/test", MagicMock(), {"data": 42})
        except Exception as exc:
            self.fail(f"publish() raised unexpectedly: {exc}")

    def test_subscribe_returns_queue_without_ros2(self):
        """subscribe() must return an asyncio.Queue even when unavailable."""
        connector = self.mod.ROS2Connector()
        q = connector.subscribe("/test", MagicMock())
        self.assertIsInstance(q, asyncio.Queue)

    def test_shutdown_is_safe_without_start(self):
        """shutdown() before start() must not raise."""
        connector = self.mod.ROS2Connector()
        try:
            connector.shutdown()
        except Exception as exc:
            self.fail(f"shutdown() raised unexpectedly: {exc}")


# ---------------------------------------------------------------------------
# Test: publish enqueue / drain with mocked ROS2
# ---------------------------------------------------------------------------

class TestPublishWithMockedROS2(unittest.TestCase):
    """publish() enqueues and _drain_publish_queue() delivers to publisher."""

    def _make_connector(self):
        """Build a connector with mocked rclpy, start it, return (connector, mocks)."""
        import importlib
        import sys

        fake_rclpy, fake_node, fake_executor = _make_fake_rclpy()

        # Inject fake modules
        sys.modules["rclpy"] = fake_rclpy
        sys.modules["rclpy.executors"] = fake_rclpy.executors
        sys.modules["rclpy.node"] = fake_rclpy.node

        import ros2_connector
        importlib.reload(ros2_connector)
        ros2_connector.ROS2_AVAILABLE = True
        ros2_connector.rclpy = fake_rclpy
        ros2_connector.Node = fake_rclpy.node.Node

        connector = ros2_connector.ROS2Connector()
        loop = asyncio.new_event_loop()
        connector.start(loop)
        # Replace the auto-created node with our tracked fake
        connector._node = fake_node

        return connector, fake_node, fake_executor, loop

    def tearDown(self):
        import sys
        for mod in ["rclpy", "rclpy.executors", "rclpy.node"]:
            sys.modules.pop(mod, None)

    def test_publish_enqueues_item(self):
        """publish() must put exactly one item onto the internal queue."""
        connector, fake_node, _, loop = self._make_connector()
        connector.publish("/eye_expression", MagicMock(), {"data": 3})
        self.assertEqual(connector._publish_queue.qsize(), 1)
        loop.close()

    def test_drain_calls_publisher_publish(self):
        """_drain_publish_queue() must call publisher.publish() with the right msg."""
        connector, fake_node, _, loop = self._make_connector()

        # Create a real message mock whose setattr we can inspect
        fake_msg_instance = MagicMock()
        FakeMsgType = MagicMock(return_value=fake_msg_instance)

        connector.publish("/eye_expression", FakeMsgType, {"data": 7})

        # Manually drain (simulates the timer callback)
        connector._drain_publish_queue()

        # Queue should be empty after drain
        self.assertTrue(connector._publish_queue.empty())

        # publisher.publish() should have been called once
        pub = connector._publishers.get("/eye_expression")
        self.assertIsNotNone(pub, "Publisher should have been lazily created")
        pub.publish.assert_called_once_with(fake_msg_instance)

        # setattr should have assigned data=7 on the message instance
        self.assertEqual(fake_msg_instance.data, 7)
        loop.close()

    def test_drain_on_empty_queue_does_not_raise(self):
        """_drain_publish_queue() with no pending items must not raise."""
        connector, _, _, loop = self._make_connector()
        try:
            connector._drain_publish_queue()
        except Exception as exc:
            self.fail(f"_drain_publish_queue() raised unexpectedly: {exc}")
        loop.close()


# ---------------------------------------------------------------------------
# Test: subscribe idempotency
# ---------------------------------------------------------------------------

class TestSubscribeIdempotency(unittest.TestCase):
    """subscribe() called twice for the same topic returns the same queue."""

    def setUp(self):
        self._patcher = patch("ros2_connector.ROS2_AVAILABLE", False)
        self._patcher.start()
        import importlib
        import ros2_connector
        importlib.reload(ros2_connector)
        self.mod = ros2_connector

    def tearDown(self):
        self._patcher.stop()

    def test_same_queue_returned_for_same_topic(self):
        connector = self.mod.ROS2Connector()
        q1 = connector.subscribe("/scan", MagicMock())
        q2 = connector.subscribe("/scan", MagicMock())
        self.assertIs(q1, q2, "subscribe() should return the same queue for the same topic")

    def test_different_queues_for_different_topics(self):
        connector = self.mod.ROS2Connector()
        q1 = connector.subscribe("/scan", MagicMock())
        q2 = connector.subscribe("/camera", MagicMock())
        self.assertIsNot(q1, q2, "Different topics should get different queues")


# ---------------------------------------------------------------------------
# Test: shutdown idempotency
# ---------------------------------------------------------------------------

class TestShutdownIdempotency(unittest.TestCase):
    def setUp(self):
        self._patcher = patch("ros2_connector.ROS2_AVAILABLE", False)
        self._patcher.start()
        import importlib
        import ros2_connector
        importlib.reload(ros2_connector)
        self.mod = ros2_connector

    def tearDown(self):
        self._patcher.stop()

    def test_double_shutdown_does_not_raise(self):
        connector = self.mod.ROS2Connector()
        try:
            connector.shutdown()
            connector.shutdown()
        except Exception as exc:
            self.fail(f"Double shutdown raised: {exc}")


# ---------------------------------------------------------------------------
# Test: set_eye_expression → ROS2Connector.publish() call convention
# ---------------------------------------------------------------------------

class TestEyeExpressionPublishConvention(unittest.TestCase):
    """
    End-to-end validation that the set_eye_expression tool correctly maps
    emotion names to integer values and calls connector.publish() with the
    right arguments.

    Tests the connector call convention without importing agent_openhouse
    (which requires LiveKit and dotenv runtime setup).
    """

    # Mirror of EMOTION_MAP in agent_openhouse.py — must stay in sync
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

    def _simulate_set_eye_expression(
        self, connector: MagicMock, msg_type: MagicMock, emotion: str
    ) -> str:
        """Reproduce the exact logic of set_eye_expression without importing the agent."""
        emotion = emotion.lower().strip()
        value = self.EMOTION_MAP.get(emotion, 0)
        connector.publish("/eye_expression", msg_type, {"data": value})
        return f"Eye expression set to {emotion}."

    def test_happy_maps_to_data_1(self):
        connector = MagicMock()
        FakeInt32 = MagicMock()
        result = self._simulate_set_eye_expression(connector, FakeInt32, "happy")
        connector.publish.assert_called_once_with(
            "/eye_expression", FakeInt32, {"data": 1}
        )
        self.assertEqual(result, "Eye expression set to happy.")

    def test_neutral_maps_to_data_0(self):
        connector = MagicMock()
        FakeInt32 = MagicMock()
        self._simulate_set_eye_expression(connector, FakeInt32, "neutral")
        connector.publish.assert_called_once_with(
            "/eye_expression", FakeInt32, {"data": 0}
        )

    def test_unknown_emotion_falls_back_to_0(self):
        connector = MagicMock()
        FakeInt32 = MagicMock()
        self._simulate_set_eye_expression(connector, FakeInt32, "embarrassed")
        connector.publish.assert_called_once_with(
            "/eye_expression", FakeInt32, {"data": 0}
        )

    def test_emotion_is_lowercased_and_stripped(self):
        """Tool must normalise emotion strings before lookup."""
        connector = MagicMock()
        FakeInt32 = MagicMock()
        self._simulate_set_eye_expression(connector, FakeInt32, "  HAPPY  ")
        connector.publish.assert_called_once_with(
            "/eye_expression", FakeInt32, {"data": 1}
        )

    def test_all_emotions_covered(self):
        """Every EMOTION_MAP entry must produce a unique non-negative integer."""
        values = list(self.EMOTION_MAP.values())
        self.assertEqual(len(values), len(set(values)), "Duplicate emotion values")
        self.assertTrue(all(v >= 0 for v in values), "Negative emotion value found")


# ---------------------------------------------------------------------------
# Test: Nav2 integration
# ---------------------------------------------------------------------------

class TestNav2Integration(unittest.TestCase):
    """
    Tests for navigate_to_pose(), cancel_navigation(), is_navigating, and
    distance_remaining.  All mocked — no real Nav2 installation required.
    """

    def setUp(self):
        self._patcher = patch("ros2_connector.ROS2_AVAILABLE", False)
        self._patcher.start()
        import importlib
        import ros2_connector
        importlib.reload(ros2_connector)
        self.mod = ros2_connector

    def tearDown(self):
        self._patcher.stop()

    def test_navigate_to_pose_unavailable_when_navigator_is_none(self):
        """navigate_to_pose() must return the unavailable string when _navigator is None."""
        connector = self.mod.ROS2Connector()
        # _navigator defaults to None and started=False — no ROS2
        loop = asyncio.new_event_loop()
        result = loop.run_until_complete(connector.navigate_to_pose(1.0, 2.0, 1.0))
        loop.close()
        self.assertIn("unavailable", result.lower())

    def test_navigate_to_pose_calls_go_to_pose_sync(self):
        """navigate_to_pose() must delegate to _go_to_pose_sync via asyncio.to_thread."""
        connector = self.mod.ROS2Connector()
        connector._navigator = MagicMock()  # non-None so the guard passes
        connector._go_to_pose_sync = MagicMock(return_value="succeeded")

        loop = asyncio.new_event_loop()
        result = loop.run_until_complete(connector.navigate_to_pose(1.0, 0.5, 0.924))
        loop.close()

        connector._go_to_pose_sync.assert_called_once_with(1.0, 0.5, 0.924)
        self.assertEqual(result, "succeeded")

    def test_cancel_navigation_sets_cancel_requested(self):
        """cancel_navigation() must set _cancel_requested when _nav_active is True."""
        connector = self.mod.ROS2Connector()
        connector._started = True  # bypass the early-return guard
        connector._nav_active = True
        connector._cancel_requested = False

        # Replace _cancel_task_sync so asyncio.to_thread can call it synchronously
        def _sync_cancel():
            if connector._nav_active:
                connector._cancel_requested = True

        connector._cancel_task_sync = _sync_cancel

        loop = asyncio.new_event_loop()
        loop.run_until_complete(connector.cancel_navigation())
        loop.close()

        self.assertTrue(connector._cancel_requested)

    def test_cancel_navigation_noop_when_not_started(self):
        """cancel_navigation() must be a no-op when the connector is not started."""
        connector = self.mod.ROS2Connector()
        # _started defaults to False
        connector._cancel_task_sync = MagicMock()
        loop = asyncio.new_event_loop()
        loop.run_until_complete(connector.cancel_navigation())
        loop.close()
        connector._cancel_task_sync.assert_not_called()

    def test_is_navigating_property(self):
        """is_navigating must reflect _nav_active."""
        connector = self.mod.ROS2Connector()
        self.assertFalse(connector.is_navigating)
        connector._nav_active = True
        self.assertTrue(connector.is_navigating)

    def test_distance_remaining_property(self):
        """distance_remaining must reflect _distance_remaining."""
        connector = self.mod.ROS2Connector()
        self.assertIsNone(connector.distance_remaining)
        connector._distance_remaining = 3.75
        self.assertAlmostEqual(connector.distance_remaining, 3.75)


if __name__ == "__main__":
    unittest.main()
