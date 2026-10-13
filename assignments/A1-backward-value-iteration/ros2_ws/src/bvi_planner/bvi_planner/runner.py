"""Shared node entry point with clean Ctrl+C / SIGTERM shutdown."""
from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException


def run_node(node_cls, args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = node_cls()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Humble race: a signal can invalidate the context mid-spin
        # (RCLError "context is not valid"). Only swallow it during shutdown.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
