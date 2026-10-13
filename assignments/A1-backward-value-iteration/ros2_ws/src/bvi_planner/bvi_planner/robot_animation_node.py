"""Animates a robot marker cell-by-cell along a nav_msgs/Path.

One instance per algorithm (BVI / ABVI). The robot starts at the start cell,
advances one cell per timer tick (1 move = cost 1), stops at the goal, holds,
and optionally restarts. A label shows elapsed cost and travel time.
"""
from __future__ import annotations

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from .markers import FRAME, hex_rgba

from .runner import run_node

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


class RobotAnimationNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_animation")
        self.label = self.declare_parameter("label", "BVI").value
        self.color = self.declare_parameter("color", "#1c7ed6").value
        rate_hz = float(self.declare_parameter("rate_hz", 2.5).value)
        self.offset = float(self.declare_parameter("lateral_offset", 0.0).value)
        self.label_dy = float(self.declare_parameter("label_offset_y", 0.9).value)
        self.loop = bool(self.declare_parameter("loop", True).value)
        hold_s = float(self.declare_parameter("hold_at_goal_s", 3.0).value)
        path_topic = self.declare_parameter("path_topic", "bvi_path").value
        marker_topic = self.declare_parameter("marker_topic", "robot_marker").value
        if not 0.2 <= rate_hz <= 20:
            raise ValueError("rate_hz must be within [0.2, 20]")

        self.dt = 1.0 / rate_hz
        self.hold_ticks = max(1, round(hold_s * rate_hz))
        self.poses: list[PoseStamped] = []
        self.index = 0
        self.hold = 0
        self.pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, marker_topic + "_pose", 10)
        self.create_subscription(Path, path_topic, self._on_path, LATCHED)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(f"[{self.label}] waiting for {path_topic} at {rate_hz} Hz")

    def _on_path(self, msg: Path) -> None:
        self.poses = list(msg.poses)
        self.index, self.hold = 0, 0
        self.get_logger().info(f"[{self.label}] path received: {len(self.poses) - 1} moves")

    def _tick(self) -> None:
        if not self.poses:
            return
        last = len(self.poses) - 1
        self._publish(self.poses[self.index], self.index, last)
        if self.index < last:
            self.index += 1
        elif self.hold == 0:
            self.get_logger().info(
                f"[{self.label}] reached goal: cost {last}, travel time {last * self.dt:.1f} s")
            self.hold = 1
        elif self.loop:
            self.hold += 1
            if self.hold > self.hold_ticks:
                self.index, self.hold = 0, 0

    def _publish(self, pose: PoseStamped, step: int, last: int) -> None:
        stamp = self.get_clock().now().to_msg()
        x = pose.pose.position.x + self.offset
        y = pose.pose.position.y + self.offset

        body = Marker()
        body.header.frame_id = FRAME
        body.header.stamp = stamp
        body.ns, body.id = f"robot_{self.label}", 0
        body.type, body.action = Marker.CYLINDER, Marker.ADD
        body.pose.position.x, body.pose.position.y, body.pose.position.z = x, y, 0.3
        body.pose.orientation.w = 1.0
        body.scale.x = body.scale.y = 0.55
        body.scale.z = 0.5
        body.color = hex_rgba(self.color)

        tag = Marker()
        tag.header = body.header
        tag.ns, tag.id = f"robot_{self.label}", 1
        tag.type, tag.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        tag.pose.position.x, tag.pose.position.y, tag.pose.position.z = x, y + self.label_dy, 1.0
        tag.pose.orientation.w = 1.0
        tag.scale.z = 0.75
        tag.color = hex_rgba(self.color)
        status = "GOAL" if step == last else f"J={last - step}"
        tag.text = f"{self.label} {step}/{last}  t={step * self.dt:.1f}s  {status}"

        out = MarkerArray()
        out.markers = [body, tag]
        self.pub.publish(out)

        pose_msg = PoseStamped()
        pose_msg.header = body.header
        pose_msg.pose = pose.pose
        self.pose_pub.publish(pose_msg)


def main(args=None) -> None:
    run_node(RobotAnimationNode, args)


if __name__ == "__main__":
    main()
