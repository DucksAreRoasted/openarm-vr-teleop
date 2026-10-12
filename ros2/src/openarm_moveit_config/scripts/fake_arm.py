#!/usr/bin/python3
"""Publish joint states and accept MoveIt trajectories without ros2_control."""

from __future__ import annotations

import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = {
    "left": [f"openarm_left_joint{index}" for index in range(1, 8)],
    "right": [f"openarm_right_joint{index}" for index in range(1, 8)],
}
READY = {
    "left": [0.0, -0.75, 0.0, 1.2, 0.0, 0.0, 0.0],
    "right": [0.0, 0.75, 0.0, 1.2, 0.0, 0.0, 0.0],
}
FINGER_JOINTS = [
    "openarm_left_finger_joint1",
    "openarm_left_finger_joint2",
    "openarm_right_finger_joint1",
    "openarm_right_finger_joint2",
]


def _seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class FakeArm(Node):
    def __init__(self) -> None:
        super().__init__("openarm_fake_arm")
        self._lock = threading.Lock()
        self._positions = {
            name: position
            for side, joints in ARM_JOINTS.items()
            for name, position in zip(joints, READY[side])
        }
        # URDF: 0 = open-ish, 0.044 = closed. Keep slightly open so default
        # start state is not finger-self-colliding if ACM is incomplete.
        for name in FINGER_JOINTS:
            self._positions[name] = 0.02
        self._publisher = self.create_publisher(JointState, "joint_states", 10)
        group = ReentrantCallbackGroup()
        self.create_timer(0.02, self._publish, callback_group=group)
        for side in ARM_JOINTS:
            ActionServer(
                self,
                FollowJointTrajectory,
                f"{side}_arm_controller/follow_joint_trajectory",
                execute_callback=self._execute,
                goal_callback=self._goal,
                cancel_callback=self._cancel,
                callback_group=group,
            )

    def _goal(self, _goal):
        return GoalResponse.ACCEPT

    def _cancel(self, _goal):
        return CancelResponse.ACCEPT

    def _publish(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        with self._lock:
            message.name = list(self._positions)
            message.position = [self._positions[name] for name in message.name]
        self._publisher.publish(message)

    def _execute(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        names = list(trajectory.joint_names)
        started = time.monotonic()
        for point in trajectory.points:
            remaining = _seconds(point.time_from_start) - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()
            with self._lock:
                for name, position in zip(names, point.positions):
                    if name in self._positions:
                        self._positions[name] = float(position)
        goal_handle.succeed()
        return FollowJointTrajectory.Result()


def main() -> None:
    rclpy.init()
    node = FakeArm()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
