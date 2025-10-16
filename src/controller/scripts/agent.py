#!/usr/bin/env python3
import mujoco
import numpy as np

class Agent:
    def __init__(self, model, data, prefix):
        # Mujoco data; pointers in C
        self.model = model
        self.data = data
        self.prefix = prefix

        # Sensor data
        self.force = []
        self.torque = []
        self.passive_joint = 0

    def applyControl(self, msg):
        self.data.ctrl[self.prefix] = msg.control
    
    def getPose(self):
        return

    def getJointAngle(self):
        for i in range(self.data.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if (self.prefix and 'paddle') in name:
                qadr = self.model.jnt_qposadr[i]
                return self.data.qpos[qadr]