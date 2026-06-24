#!/usr/bin/env python3

import math
import os
import importlib
from typing import List

import mujoco
import rclpy
from actuator_msgs.msg import Actuators
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    f"leg_{leg}_base__leg_{leg}a" if joint == 0 else
    f"leg_{leg}a__leg_{leg}b" if joint == 1 else
    f"leg_{leg}b__leg_{leg}c"
    for leg in range(6)
    for joint in range(3)
]

NEUTRAL_JOINT_POSITIONS = [
    0.690, 0.643, 1.888,
    0.000, 0.617, 1.669,
    -0.690, 0.643, 1.888,
    0.690, -0.643, -1.888,
    0.000, -0.617, -1.669,
    -0.690, -0.643, -1.888,
]


def seconds_to_time(seconds: float) -> Time:
    msg = Time()
    msg.sec = int(math.floor(seconds))
    msg.nanosec = int((seconds - msg.sec) * 1_000_000_000)
    return msg


class MujocoSimulator(Node):
    def __init__(self):
        super().__init__("mujoco_simulator")

        default_model_path = os.path.join(
            os.environ.get("AMENT_PREFIX_PATH", "").split(":")[0],
            "share",
            "antsy_simulation",
            "mujoco",
            "antsy.xml",
        )
        if not os.path.exists(default_model_path):
            default_model_path = os.path.join(
                os.getcwd(), "src", "antsy_simulation", "mujoco", "antsy.xml")

        self.declare_parameter("model_path", default_model_path)
        self.declare_parameter("run_headless", True)
        self.declare_parameter("realtime_factor", 1.0)
        self.declare_parameter("state_publish_rate", 50.0)

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.run_headless = self.get_parameter("run_headless").get_parameter_value().bool_value
        self.realtime_factor = (
            self.get_parameter("realtime_factor").get_parameter_value().double_value)
        self.state_publish_period = 1.0 / max(
            1.0,
            self.get_parameter("state_publish_rate").get_parameter_value().double_value,
        )

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.joint_qpos_addr = self._joint_addresses("qposadr")
        self.joint_dof_addr = self._joint_addresses("dofadr")
        self.actuator_ids = self._actuator_ids()
        self.target_positions = list(NEUTRAL_JOINT_POSITIONS)
        self.last_state_publish_time = -self.state_publish_period

        self._set_initial_pose()

        self.clock_pub = self.create_publisher(Clock, "clock", 10)
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.create_subscription(Actuators, "actuators", self.actuator_callback, 10)

        self.viewer = None
        if not self.run_headless:
            mujoco_viewer = importlib.import_module("mujoco.viewer")
            self.viewer = mujoco_viewer.launch_passive(self.model, self.data)

        timer_period = max(self.model.opt.timestep / max(self.realtime_factor, 0.01), 0.001)
        self.timer = self.create_timer(timer_period, self.step)
        self.get_logger().info(f"MuJoCo simulation loaded: {model_path}")

    def _joint_addresses(self, field: str) -> List[int]:
        address_array = getattr(self.model, f"jnt_{field}")
        addresses = []
        for name in JOINT_NAMES:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise RuntimeError(f"MuJoCo model is missing joint {name}")
            addresses.append(int(address_array[joint_id]))
        return addresses

    def _actuator_ids(self) -> List[int]:
        actuator_ids = []
        for name in JOINT_NAMES:
            actuator_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_pos")
            if actuator_id < 0:
                raise RuntimeError(f"MuJoCo model is missing actuator {name}_pos")
            actuator_ids.append(actuator_id)
        return actuator_ids

    def _set_initial_pose(self):
        self.data.qpos[0:7] = [0.0, 0.0, 0.105, 1.0, 0.0, 0.0, 0.0]
        for qpos_addr, position in zip(self.joint_qpos_addr, self.target_positions):
            self.data.qpos[qpos_addr] = position
        for actuator_id, position in zip(self.actuator_ids, self.target_positions):
            self.data.ctrl[actuator_id] = position
        mujoco.mj_forward(self.model, self.data)

    def actuator_callback(self, msg: Actuators):
        if len(msg.position) < len(JOINT_NAMES):
            self.get_logger().warn(
                f"Ignoring actuator command with {len(msg.position)} positions; "
                f"expected {len(JOINT_NAMES)}.")
            return
        self.target_positions = list(msg.position[:len(JOINT_NAMES)])

    def step(self):
        for actuator_id, position in zip(self.actuator_ids, self.target_positions):
            self.data.ctrl[actuator_id] = position

        mujoco.mj_step(self.model, self.data)

        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()

        clock_msg = Clock()
        clock_msg.clock = seconds_to_time(float(self.data.time))
        self.clock_pub.publish(clock_msg)

        if self.data.time - self.last_state_publish_time >= self.state_publish_period:
            self.last_state_publish_time = float(self.data.time)
            self.publish_joint_states(clock_msg.clock)
            self.publish_odom(clock_msg.clock)

    def publish_joint_states(self, stamp: Time):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(JOINT_NAMES)
        msg.position = [float(self.data.qpos[address]) for address in self.joint_qpos_addr]
        msg.velocity = [float(self.data.qvel[address]) for address in self.joint_dof_addr]
        self.joint_state_pub.publish(msg)

    def publish_odom(self, stamp: Time):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"

        msg.pose.pose.position.x = float(self.data.qpos[0])
        msg.pose.pose.position.y = float(self.data.qpos[1])
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = float(self.data.qpos[3])
        msg.pose.pose.orientation.x = float(self.data.qpos[4])
        msg.pose.pose.orientation.y = float(self.data.qpos[5])
        msg.pose.pose.orientation.z = float(self.data.qpos[6])

        msg.twist.twist.linear.x = float(self.data.qvel[0])
        msg.twist.twist.linear.y = float(self.data.qvel[1])
        msg.twist.twist.linear.z = float(self.data.qvel[2])
        msg.twist.twist.angular.x = float(self.data.qvel[3])
        msg.twist.twist.angular.y = float(self.data.qvel[4])
        msg.twist.twist.angular.z = float(self.data.qvel[5])
        self.odom_pub.publish(msg)


def main():
    rclpy.init()
    node = MujocoSimulator()
    try:
        rclpy.spin(node)
    finally:
        if node.viewer is not None:
            node.viewer.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
