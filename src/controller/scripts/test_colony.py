#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math, time, pathlib
from dataclasses import dataclass
from typing import List, Tuple
import numpy as np
import mujoco
import mujoco.viewer

# ===== User Settings =====
N              = 8               # Number of agents/bots
RADIUS         = 0.15            # [m] Radius of the circle (agents placed at equal intervals)
FACE_RADIAL    = True            # True: Agents face outwards (radial direction)
YAW_OFFSET_DEG = 0.0             # Fine adjustment of yaw for all agents (degrees)
YAW_OFFSET     = math.radians(YAW_OFFSET_DEG)

OMEGA          = 2.0 * math.pi * 2.0 # [rad/s] Base angular velocity (= 2 Hz)
KAPPA          = 0.5               # κ: Sensor coefficient gain
MIN_DELPHI     = 0.1               # Lower limit for φ̇ (to prevent stopping)
HEADLESS       = False
SIM_TIME       = 10.0              # Used only when HEADLESS is True

# Cosine scan limits (degrees -> radians)
MIN_DEG = -45.0
MAX_DEG =  45.0
MIN_RAD = math.radians(MIN_DEG)
MAX_RAD = math.radians(MAX_DEG)
MID_RAD = 0.5 * (MIN_RAD + MAX_RAD)
HALF_SW = 0.5 * (MAX_RAD - MIN_RAD)

# Add a dummy mass to avoid mass-0 error when 'colony' joint is free
ADD_COLONY_DUMMY_MASS = True

@dataclass
class Agent:
    prefix: str            # 'r1_', 'r2_', ...
    jid_motor: int         # motor_hinge joint ID
    jid_paddle: int        # paddle_hinge joint ID
    aid_motor: int         # Position actuator ID (r?_motor)
    phi: float = 0.0       # Internal phase
    xi: float  = 0.0       # Normalized paddle relative angle

# ---------- Generate Concentric Circle Configuration XML: test.xml ----------
def generate_test_xml(n: int, radius: float, xml_dir: pathlib.Path) -> pathlib.Path:
    """ Generates models/xml/test_N_circle.xml and returns its path """

    def yaw_to_quat(yaw: float) -> str:
        # Yaw around z-axis (MuJoCo quat is w x y z)
        w = math.cos(yaw/2.0); z = math.sin(yaw/2.0)
        return f"{w} 0 0 {z}"

    bots_xml = []
    for i in range(n):
        ang = 2*math.pi * i / n
        x = radius * math.cos(ang)
        y = radius * math.sin(ang)

        if FACE_RADIAL:
            # ★ Radial direction (outward) + offset
            #   Adding +π/2 to position angle atan2(y,x) means
            #   at (0, -R), yaw=0 -> quat="1 0 0 0" (Matches your example)
            yaw = math.atan2(y, x) + math.pi/2 + YAW_OFFSET
            quat = yaw_to_quat(yaw)
        else:
            quat = "1 0 0 0"  # Keep orientation as is

        idx = i + 1
        bots_xml.append(
f"""      <body name="bot{idx}" pos="{x:.6f} {y:.6f} 0" quat="{quat}">
        <attach model="modbot" body="main_body" prefix="r{idx}_"/>
      </body>"""
        )

    # Transparent dummy mass (To avoid mass-0 error when using a free joint on the colony)
    dummy = ""
    if ADD_COLONY_DUMMY_MASS:
        dummy = ('      <geom type="sphere" size="0.01" density="1000" '
                 'rgba="0 0 0 0" contype="0" conaffinity="0"/>\n')

    xml_text = f"""<mujoco model="modular">

  <option timestep="0.001" gravity="0 0 0" integrator="implicit"
           density="1025" viscosity="0.001" iterations="50" solver="Newton"/>
  <include file="environment.xml"/>

  <asset>
    <model name="modbot" file="modular.xml"/>
  </asset>

  <worldbody>
    <body name="colony" pos="0 0 0.5">
      <site name="COM" pos="0 0 0"/>
      <joint name="base" type="free"/>
{dummy}{"".join(bots_xml)}
    </body>
  </worldbody>

  <sensor>
    <force  name="colony_force"  site="COM"/>
    <torque name="colony_torque" site="COM"/>
  </sensor>

</mujoco>
"""
    out = xml_dir / f"test_{n}_circle.xml"
    out.write_text(xml_text)
    return out

# ---------- Model Loading ----------
def load_model() -> Tuple[mujoco.MjModel, mujoco.MjData, pathlib.Path]:
    root = pathlib.Path(__file__).resolve().parents[3]
    xml_dir = root / "models" / "xml"
    # Generate and load the model
    test_path = generate_test_xml(N, RADIUS, xml_dir)
    m = mujoco.MjModel.from_xml_path(str(test_path))
    d = mujoco.MjData(m)
    return m, d, root

# ---------- Agent Detection (Automatic prefix) ----------
def detect_agents(m: mujoco.MjModel) -> List[Agent]:
    prefs = []
    for j in range(m.njnt):
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or ""
        if len(nm) >= 3 and nm[0]=="r" and nm[1].isdigit() and nm[2]=="_":
            pf = nm[:3]  # 'r1_', 'r2_', ...
            if pf not in prefs:
                prefs.append(pf)
    agents: List[Agent] = []
    for pf in prefs:
        jid_m = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT,     f"{pf}motor_hinge")
        jid_p = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT,     f"{pf}paddle_hinge")
        aid_m = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{pf}motor")
        if jid_m == -1 or aid_m == -1:
            continue
        agents.append(Agent(prefix=pf, jid_motor=jid_m, jid_paddle=jid_p, aid_motor=aid_m,
                             phi=np.random.uniform(-math.pi, math.pi)))
    return agents

# ---------- Sensor/Phase/Desired Angle ----------
def get_angles(m: mujoco.MjModel, d: mujoco.MjData, ag: Agent):
    q_m = d.qpos[m.jnt_qposadr[ag.jid_motor]]
    if ag.jid_paddle != -1:
        q_p = d.qpos[m.jnt_qposadr[ag.jid_paddle]]
        lo, hi = m.jnt_range[ag.jid_paddle]
        amax = hi - lo
    else:
        q_p, amax = 0.0, math.radians(45.0)
    return float(q_m), float(q_p), float(amax)

def compute_xi(alpha_rel: float, alpha_max: float) -> float:
    return float(np.clip(alpha_rel / max(alpha_max, 1e-8), 0.0, 1.0))

def integrate_phi(phi: float, xi: float, dt: float) -> float:
    dphi = OMEGA * (1.0 + KAPPA * xi * math.cos(phi))
    if dphi < MIN_DELPHI:
        dphi = MIN_DELPHI
    phi += dphi * dt
    return (phi + math.pi) % (2*math.pi) - math.pi

def desired_angle_from_phi(phi: float) -> float:
    # Cosine scan: θ_ref = MID - HALF * cos(phi)
    return MID_RAD - HALF_SW * math.cos(phi)

# ---------- Execution ----------
def run():
    m, d, _ = load_model()
    agents = detect_agents(m)
    if not agents:
        raise RuntimeError("Could not find r*_motor / r*_motor_hinge. Please check XML prefix/actuator names.")
    dt = m.opt.timestep

    if HEADLESS:
        t0 = time.time()
        while time.time() - t0 < SIM_TIME:
            d.ctrl[:] = 0.0
            for ag in agents:
                _, alpha_rel, alpha_max = get_angles(m, d, ag)
                ag.xi  = compute_xi(alpha_rel, alpha_max)
                ag.phi = integrate_phi(ag.phi, ag.xi, dt)
                d.ctrl[ag.aid_motor] = desired_angle_from_phi(ag.phi)
            mujoco.mj_step(m, d)
        return

    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            d.ctrl[:] = 0.0
            for ag in agents:
                _, alpha_rel, alpha_max = get_angles(m, d, ag)
                ag.xi  = compute_xi(alpha_rel, alpha_max)
                ag.phi = integrate_phi(ag.phi, ag.xi, dt)
                d.ctrl[ag.aid_motor] = desired_angle_from_phi(ag.phi)
            mujoco.mj_step(m, d)
            viewer.sync()
            time.sleep(0.001)

if __name__ == "__main__":
    run()