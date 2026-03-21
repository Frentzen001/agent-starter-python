"""
tests/test_ros2_connector_level2.py
=====================================
Level 2 integration tests for ros2_connector.ROS2Connector.

These tests use a REAL rclpy context — no mocking.  They verify:
  - A real publisher is created and the drain timer delivers messages
  - publish() → subscribe() loopback: a message published on topic X
    actually arrives in the asyncio.Queue returned by subscribe(X)
  - Multiple messages are all drained without hang
  - connector repr updates after publish/subscribe

Requirements
------------
Run with ROS 2 sourced so rclpy and std_msgs are on PYTHONPATH:

    source /opt/ros/humble/setup.bash
    PYTHONPATH=src uv run pytest tests/test_ros2_connector_level2.py -v

Tests are skipped gracefully when rclpy is not importable.
"""

from __future__ import annotations

import asyncio
import sys
import unittest

# ---------------------------------------------------------------------------
# Skip the entire module if rclpy is not available
# ---------------------------------------------------------------------------
try:
    import rclpy  # noqa: F401
    from std_msgs.msg import Int32, String

    RCLPY_AVAILABLE = True
except ImportError:
    RCLPY_AVAILABLE = False

if not RCLPY_AVAILABLE:
    # Mark every test in this module as skipped at collection time
    import pytest
    pytestmark = pytest.mark.skip(reason="rclpy not available — source ROS 2 first")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _get_connector():
    """Import and return a fresh ROS2Connector (module reloaded so state is clean)."""
    import importlib
    import ros2_connector
    importlib.reload(ros2_connector)
    return ros2_connector.ROS2Connector()


# ---------------------------------------------------------------------------
# Base class: handles rclpy init/shutdown around each test
# ---------------------------------------------------------------------------

class ROS2TestBase(unittest.TestCase):
    def setUp(self):
        import rclpy as _rclpy
        if not _rclpy.ok():
            _rclpy.init()
        self.connector = _get_connector()
        self.loop = asyncio.new_event_loop()
        self.connector.start(self.loop)

    def tearDown(self):
        self.connector.shutdown()
        self.loop.close()
        import rclpy as _rclpy
        try:
            if _rclpy.ok():
                _rclpy.shutdown()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

@unittest.skipUnless(RCLPY_AVAILABLE, "rclpy not available")
class TestRealPublisher(ROS2TestBase):
    """Verify that publish() causes a real rclpy publisher to be lazily created
    and the drain timer empties the queue within 200 ms."""

    def test_publisher_created_after_drain(self):
        """After calling publish() and waiting for the drain timer, the publisher
        must exist in _publishers and the internal queue must be empty."""
        self.connector.publish("/level2_test_pub", Int32, {"data": 99})

        # The drain timer fires every 10 ms — wait 200 ms to be safe
        self.loop.run_until_complete(asyncio.sleep(0.2))

        self.assertIn("/level2_test_pub", self.connector._publishers,
                      "Publisher should be lazily created by the drain timer")
        self.assertTrue(self.connector._publish_queue.empty(),
                        "Publish queue should be empty after drain")

    def test_multiple_messages_all_drained(self):
        """Publishing N messages must drain all of them without hang."""
        N = 10
        for i in range(N):
            self.connector.publish("/level2_multi", Int32, {"data": i})

        self.loop.run_until_complete(asyncio.sleep(0.3))

        self.assertTrue(self.connector._publish_queue.empty(),
                        f"All {N} messages should be drained")

    def test_publish_different_topics_create_separate_publishers(self):
        """Each distinct topic must get its own rclpy publisher."""
        self.connector.publish("/topic_a", Int32, {"data": 1})
        self.connector.publish("/topic_b", Int32, {"data": 2})

        self.loop.run_until_complete(asyncio.sleep(0.2))

        self.assertIn("/topic_a", self.connector._publishers)
        self.assertIn("/topic_b", self.connector._publishers)
        self.assertIsNot(
            self.connector._publishers["/topic_a"],
            self.connector._publishers["/topic_b"],
            "Each topic should have a distinct publisher object",
        )


@unittest.skipUnless(RCLPY_AVAILABLE, "rclpy not available")
class TestRealPubSubLoopback(ROS2TestBase):
    """
    Publish a message and verify it is delivered to the asyncio.Queue
    returned by subscribe() — the full thread-safe bridge path.

    Relies on DDS intra-process delivery (same process, same DDS participant)
    which is typically < 50 ms.  We allow up to 2 s before failing.
    """

    TOPIC = "/level2_loopback"
    TIMEOUT_SEC = 2.0

    def _run(self, coro):
        return self.loop.run_until_complete(coro)

    def test_published_int32_arrives_in_queue(self):
        """An Int32 published via connector.publish() must arrive in the
        asyncio.Queue returned by connector.subscribe()."""

        async def _test():
            q = self.connector.subscribe(self.TOPIC, Int32)

            # Small delay so DDS can discover the publisher/subscriber pair
            await asyncio.sleep(0.1)
            self.connector.publish(self.TOPIC, Int32, {"data": 42})

            msg = await asyncio.wait_for(q.get(), timeout=self.TIMEOUT_SEC)
            return msg

        msg = self._run(_test())
        self.assertIsNotNone(msg, "Should have received a message")
        self.assertEqual(msg.data, 42,
                         f"Expected data=42, got data={msg.data}")

    def test_multiple_messages_delivered_in_order(self):
        """Three sequential publishes must arrive in the queue in order."""

        async def _test():
            q = self.connector.subscribe(self.TOPIC + "_order", Int32)
            await asyncio.sleep(0.1)

            for val in [1, 2, 3]:
                self.connector.publish(self.TOPIC + "_order", Int32, {"data": val})
                await asyncio.sleep(0.05)  # give each message time to drain + deliver

            results = []
            for _ in range(3):
                msg = await asyncio.wait_for(q.get(), timeout=self.TIMEOUT_SEC)
                results.append(msg.data)
            return results

        results = self._run(_test())
        self.assertEqual(results, [1, 2, 3],
                         f"Messages should arrive in publish order, got {results}")

    def test_subscribe_idempotent_with_real_node(self):
        """Calling subscribe() twice for the same topic on a live connector
        must return the same asyncio.Queue object."""
        q1 = self.connector.subscribe(self.TOPIC + "_idem", Int32)
        q2 = self.connector.subscribe(self.TOPIC + "_idem", Int32)
        self.assertIs(q1, q2)


@unittest.skipUnless(RCLPY_AVAILABLE, "rclpy not available")
class TestConnectorLifecycle(ROS2TestBase):
    """Lifecycle sanity checks with a real rclpy context."""

    def test_repr_updates_after_publish(self):
        """repr() should list the published topic after a drain."""
        self.connector.publish("/repr_topic", Int32, {"data": 0})
        self.loop.run_until_complete(asyncio.sleep(0.2))
        r = repr(self.connector)
        self.assertIn("started", r)
        self.assertIn("/repr_topic", r)

    def test_started_flag_is_true_after_start(self):
        self.assertTrue(self.connector._started)

    def test_started_flag_is_false_after_shutdown(self):
        self.connector.shutdown()
        self.assertFalse(self.connector._started)
        # repair for tearDown
        self.connector._started = False


if __name__ == "__main__":
    unittest.main()
