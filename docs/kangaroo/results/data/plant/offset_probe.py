"""Steady offset of PAL's arm trajectory controllers under gravity (plan Task 2, step 3a).

Sends each arm to PAL home in 2 s, lets 3 s of sim time pass (sim running in real time, not paused), then
averages reference - measured over 1 s from /arm_<side>_controller/controller_state.  Prints JSON.
"""
import json, sys, time
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

HOME = {"left": [0.239, 1.321, 1.57, 0.8], "right": [-0.239, 1.321, -1.57, 0.8]}   # SO-2 joint_setup.py:16
rclpy.init()
node = rclpy.create_node("offset_probe", parameter_overrides=[Parameter("use_sim_time", value=True)])
samples = {s: [] for s in HOME}
pubs = {s: node.create_publisher(JointTrajectory, "/arm_%s_controller/joint_trajectory" % s, 1) for s in HOME}
for s in HOME:
    node.create_subscription(JointTrajectoryControllerState, "/arm_%s_controller/controller_state" % s,
                             lambda m, s=s: samples[s].append((node.get_clock().now().nanoseconds * 1e-9,
                                                               np.array(m.reference.positions) - np.array(m.feedback.positions))), 10)
def spin_sim(seconds):
    rclpy.spin_once(node, timeout_sec=0.1)
    end = node.get_clock().now() + Duration(seconds=seconds)
    while node.get_clock().now() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
spin_sim(0.5)
for s in HOME:
    while pubs[s].get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    msg = JointTrajectory(joint_names=["arm_%s_%d_joint" % (s, k) for k in range(1, 5)])
    msg.points.append(JointTrajectoryPoint(positions=HOME[s], velocities=[0.0] * 4, time_from_start=Duration(seconds=2.0).to_msg()))
    pubs[s].publish(msg)
spin_sim(5.0)
t_end = node.get_clock().now().nanoseconds * 1e-9
out = {"pose": HOME, "window_s": [t_end - 1.0, t_end]}
for s in HOME:
    last = np.array([d for t, d in samples[s] if t >= t_end - 1.0])
    out[s] = {"samples": int(len(last)), "mean_offset_rad": last.mean(axis=0).tolist(), "max_abs_offset_rad": np.abs(last).max(axis=0).tolist()}
print(json.dumps(out, indent=1))
