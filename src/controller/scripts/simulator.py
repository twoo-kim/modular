#!/usr/bin/env python3
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import time
import yaml
import numpy as np

from modular_msgs.msg import StateMsg, ControlMsg
from agent import Agent

class MujocoSimulator(Node):
    def __init__(self):
        super().__init__('mujoco_simulator')

        # Get Parameters
        self.declare_parameter('config_path', 'default')
        self.loadYAML(self.get_parameter('config_path').get_parameter_value().string_value)

        # Mujoco model
        self.model = mujoco.MjModel.from_xml_path(self.sim_path)
        self.data = mujoco.MjData(self.model)
        self.data.ctrl[:] = [0]*len(self.data.ctrl)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        # Get multiple robots
        self.agents = {i: Agent(self.model, self.data, i) for i in range(self.N)}
        
        # Subscriber
        self.control_subs = []
        for n in range(self.N):
            topic = f'/control_topic{n}'
            self.control_sub.append(
                self.create_subscription(ControlMsg, topic, self.controlCallback(n), 10)
            )
        
        # Publisher
        self.pose_pub = self.create_publisher(StateMsg, '/pose_topic', 10)

    def loadYAML(self, path):
        with open(path, 'r') as f:
            config = yaml.safe_load(f)
        self.sim_path = config['mujoco_xml_path']
        self.N = config['N']

    def controlCallback(self, n):    
        def callback(msg):
            self.agents[n].applyControl(msg)
        return callback

    def publishPose(self):
        # Publish ground truth in simulation
        msg = StateMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame

        # Floating base pose
        base_id = self.model.body(self.base_frame).id
        msg.base_pose.position.x = self.data.xpos[base_id][0]
        msg.base_pose.position.y = self.data.xpos[base_id][1]
        msg.base_pose.position.z = self.data.xpos[base_id][2]
        msg.base_pose.orientation.w = self.data.xquat[base_id][0]
        msg.base_pose.orientation.x = self.data.xquat[base_id][1]
        msg.base_pose.orientation.y = self.data.xquat[base_id][2]
        msg.base_pose.orientation.z = self.data.xquat[base_id][3]

        # Floating base twist
        base_twist = self.data.cvel[base_id]
        msg.base_twist.angular.x = base_twist[0]
        msg.base_twist.angular.y = base_twist[1]
        msg.base_twist.angular.z = base_twist[2]
        msg.base_twist.linear.x = base_twist[3]
        msg.base_twist.linear.y = base_twist[4]
        msg.base_twist.linear.z = base_twist[5]

        # End-effector pose
        ee_id = self.model.body(self.ee_frame).id
        msg.ee_pose.position.x = self.data.xpos[ee_id][0]
        msg.ee_pose.position.y = self.data.xpos[ee_id][1]
        msg.ee_pose.position.z = self.data.xpos[ee_id][2]
        msg.ee_pose.orientation.w = self.data.xquat[ee_id][0]
        msg.ee_pose.orientation.x = self.data.xquat[ee_id][1]
        msg.ee_pose.orientation.y = self.data.xquat[ee_id][2]
        msg.ee_pose.orientation.z = self.data.xquat[ee_id][3]

        # End-effector twist
        ee_twist = self.data.cvel[ee_id]
        msg.ee_twist.angular.x = ee_twist[0]
        msg.ee_twist.angular.y = ee_twist[1]
        msg.ee_twist.angular.z = ee_twist[2]
        msg.ee_twist.linear.x = ee_twist[3]
        msg.ee_twist.linear.y = ee_twist[4]
        msg.ee_twist.linear.z = ee_twist[5]

        # Joint positions and velocities
        for j_id in range(self.model.njnt):
            j_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j_id)
            if ('propeller' in j_name): continue
            j_type = self.model.jnt_type[j_id]
            j_qadr = self.model.jnt_qposadr[j_id]
            j_dadr = self.model.jnt_dofadr[j_id]
            if j_type in [mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE]:
                msg.joint_names.append(j_name)
                msg.joint_pos.append(self.data.qpos[j_qadr])
                msg.joint_vel.append(self.data.qvel[j_dadr])
        
        self.pose_pub.publish(msg)

    def run(self):
        exec = SingleThreadedExecutor()
        exec.add_node(self)

        while (self.viewer.is_running() and rclpy.ok()):
            # 1. Spin once
            exec.spin_once(timeout_sec = 0.0)

            # 2. Forward step
            mujoco.mj_step(self.model, self.data)

            # 3. Publish pose
            self.publishPose()

            # 4. Render
            self.viewer.sync()
            time.sleep(0.01)
            
        exec.shutdown()
        self.destroy_node()
        rclpy.shutdown()

# Run simulation node
if __name__ == "__main__":
    rclpy.init()
    sim = MujocoSimulator()
    sim.run()
