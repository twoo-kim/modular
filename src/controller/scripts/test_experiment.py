#!/usr/bin/env python3
import os
import math
import numpy as np
import mujoco
import mujoco.viewer
import matplotlib.pyplot as plt


class MuJoCoRobotSim:
  def __init__(
    self,
    model_path: str,
    keyframe_name: str = "init_pose",
    step: float = 0.001,
    sim_time: float = 10.0,
    amp: float = math.pi / 6,
    freq: float = 1.0,
    motor_name_substr: str = "motor",
    paddle_name_substr: str = "paddle",
    force_sensor_substr: str = "force",
    robot_body_substrs=("colony", "bot"),
    system_body_name: str = "force_sensor",
    passive_viewer: bool = True,
  ):
    self.model_path = os.path.abspath(model_path)
    self.keyframe_name = keyframe_name
    self.step = float(step)
    self.sim_time = float(sim_time)
    self.amp = float(amp)
    self.freq = float(freq)
    self.omega = 2.0 * math.pi * self.freq  # rad/s (continuous-time)

    self.motor_name_substr = motor_name_substr
    self.paddle_name_substr = paddle_name_substr
    self.force_sensor_substr = force_sensor_substr
    self.robot_body_substrs = tuple(robot_body_substrs)
    self.system_body_name = system_body_name
    self.passive_viewer = passive_viewer

    self.model = mujoco.MjModel.from_xml_path(self.model_path)
    self.data = mujoco.MjData(self.model)

    # Filled by _collect_ids()
    self.robot_body_ids = []
    self.motor_jnt_ids = []
    self.paddle_jnt_ids = []
    self.force_sensor_ids = []
    self.system_body_id = None

    # Data buffers (filled during run)
    self.t = []
    self.ang_motor = {}      # jnt_id -> [rad]
    self.ang_paddle = {}     # jnt_id -> [rad]
    self.force = {}          # sensor_id -> {'x':[], 'y':[], 'z':[]}
    self.orientation = {}    # body_id -> 3x3 (last sample)
    self.init_pos = None

    # Experiment data holders (loaded later)
    # Structure after load: {'t': ndarray, 'passive_deg': ndarray|None, 'active_deg': ndarray|None, 'angle_deg': ndarray|None}
    # 'angle_deg' kept for backward compatibility (maps to passive_deg if present)
    self.exp_angles = None
    self.exp_force = None    # dict with 't', and 'fz_N'

    # Aligned series
    self.aligned = {}

    self._reset_to_keyframe()
    self._collect_ids()
    self._capture_init_pos()

  # ---------------------------- Setup helpers ----------------------------

  def _reset_to_keyframe(self):
    key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, self.keyframe_name)
    if key_id < 0:
        raise ValueError(f'Keyframe "{self.keyframe_name}" not found in model.')
    mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
    mujoco.mj_forward(self.model, self.data)

  def _collect_ids(self):
    # System body
    self.system_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.system_body_name)
    if self.system_body_id < 0:
        raise ValueError(f'Body "{self.system_body_name}" not found in model.')

    # Robot bodies (for orientation tracking, etc.)
    self.robot_body_ids.clear()
    for b in range(self.model.nbody):
        name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b) or ""
        if any(substr in name for substr in self.robot_body_substrs):
            self.robot_body_ids.append(b)

    # Joints
    self.motor_jnt_ids.clear()
    self.paddle_jnt_ids.clear()
    for j in range(self.model.njnt):
        name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j) or ""
        if self.motor_name_substr in name:
            self.motor_jnt_ids.append(j)
        elif self.paddle_name_substr in name:
            self.paddle_jnt_ids.append(j)

    # Force sensors
    self.force_sensor_ids.clear()
    for s in range(self.model.nsensor):
        name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SENSOR, s) or ""
        if self.force_sensor_substr in name:
            self.force_sensor_ids.append(s)

  def _capture_init_pos(self):
    self.init_pos = self.data.xpos[self.system_body_id].copy()

  # ---------------------------- Math helpers ----------------------------

  @staticmethod
  def quaternion_to_matrix(q):
    """MuJoCo quaternion is (w, x, y, z)."""
    w, x, y, z = q
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n == 0:
        return np.eye(3)
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),       2*(x*z + y*w)],
        [2*(x*y + z*w),         1 - 2*(x*x + z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),         2*(y*z + x*w),       1 - 2*(x*x + y*y)],
    ])

  # ---------------------------- Sampling helpers ----------------------------

  def _read_joint_angle(self, jnt_id: int) -> float:
    qadr = self.model.jnt_qposadr[jnt_id]
    return float(self.data.qpos[qadr])

  def _read_sensor_vec3(self, sensor_id: int) -> np.ndarray:
    """ Read sensor from sensordata using sensor address + dim. """
    adr = int(self.model.sensor_adr[sensor_id])
    dim = int(self.model.sensor_dim[sensor_id])
    if dim != 3:
      raise ValueError(f"Sensor {sensor_id} dim={dim}, expected 3.")
    return np.array(self.data.sensordata[adr:adr + dim], dtype=float)

  # ---------------------------- Control + run ----------------------------

  def control(self, t: float):
    self.data.ctrl[:] = self.amp * math.sin(self.omega * t)

  def step_once(self, t: float):
    self.control(t)
    mujoco.mj_step(self.model, self.data)
    self._record(t)

  def _record(self, t: float):
    self.t.append(t)

    # Joints
    for jnt_id in self.motor_jnt_ids:
      ang = self._read_joint_angle(jnt_id)
      self.ang_motor.setdefault(jnt_id, []).append(ang)

    for jnt_id in self.paddle_jnt_ids:
      ang = self._read_joint_angle(jnt_id)
      self.ang_paddle.setdefault(jnt_id, []).append(ang)
    
    for body_id in self.robot_body_ids:
      self.orientation[body_id] = self.quaternion_to_matrix(self.data.xquat[body_id])

    # Forces
    for sensor_id in self.force_sensor_ids:
      f = -self._read_sensor_vec3(sensor_id)
      buf = self.force.setdefault(sensor_id, {"x": [], "y": [], "z": []})
      buf["x"].append(float(f[0]))
      buf["y"].append(float(f[1]))
      buf["z"].append(float(f[2]))

  def run(self):
    n_steps = int(self.sim_time / self.step)
    if self.passive_viewer:
      with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
        for k in range(n_steps):
          t = (k + 1) * self.step
          self.step_once(t)
          if not viewer.is_running():
            break
          # viewer.sync()  # optional
    else:
      for k in range(n_steps):
        t = (k + 1) * self.step
        self.step_once(t)

  # ---------------------------- Reporting ----------------------------

  def plot_basic(self, motor_index: int = 0, paddle_index: int = 0, force_sensor_index: int = 0):
    # Angles
    if self.motor_jnt_ids and motor_index < len(self.motor_jnt_ids):
      mid = self.motor_jnt_ids[motor_index]
      a_motor = np.degrees(self.ang_motor.get(mid, []))
    else:
      a_motor = None

    if self.paddle_jnt_ids and paddle_index < len(self.paddle_jnt_ids):
      pid = self.paddle_jnt_ids[paddle_index]
      a_paddle = np.degrees(self.ang_paddle.get(pid, []))
    else:
      a_paddle = None

    if a_motor is not None or a_paddle is not None:
      plt.figure()
      if a_motor is not None:
        plt.plot(self.t, a_motor, label="motor angle")
      if a_paddle is not None:
        plt.plot(self.t, a_paddle, label="paddle angle")
      plt.xlabel("Time [s]")
      plt.ylabel("Angle [deg]")
      plt.title("Joint Angle vs Time")
      plt.legend()
      plt.grid(True)

    # Force
    if self.force_sensor_ids and force_sensor_index < len(self.force_sensor_ids):
      sid = self.force_sensor_ids[force_sensor_index]
      sname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SENSOR, sid)
      fx = self.force[sid]["x"]
      fy = self.force[sid]["y"]
      fz = self.force[sid]["z"]

      plt.figure()
      plt.plot(self.t, fz, label=f"{sname} z")
      plt.xlabel("Time [s]")
      plt.ylabel("Force [N]")
      plt.title("Force vs Time")
      plt.legend()
      plt.grid(True)

    plt.show()

# ============================ CSV + alignment ============================
  @staticmethod
  def _load_csv_columns(path: str, required_cols: list[str]) -> dict[str, np.ndarray]:
    """
    Small CSV loader without pandas.
    Assumes:
      - comma-separated
      - header row present
      - numeric columns (except any optional string cols we ignore)
    """
    path = os.path.abspath(path)
    with open(path, "r", encoding="utf-8") as f:
      header = f.readline().strip().split(",")

    col_idx = {}
    for c in required_cols:
      if c not in header:
        raise ValueError(f"CSV '{path}' missing required column '{c}'. Found: {header}")
      col_idx[c] = header.index(c)

    # Load all rows
    data = {c: [] for c in required_cols}
    with open(path, "r", encoding="utf-8") as f:
      _ = f.readline()  # header
      for line in f:
        line = line.strip()
        if not line:
          continue
        parts = line.split(",")
        for c in required_cols:
          data[c].append(float(parts[col_idx[c]]))

    return {c: np.asarray(v, dtype=float) for c, v in data.items()}

  def load_experiment_angles_csv(
      self,
      path: str,
      time_col: str = "time_shifted",
      angle_col: str = "q_deg",
      active_angle_col: str | None = None,
  ):
    """
    Load experimental angles. Supports both passive and active joint angles when available.
    - angle_col: interpreted as the passive angle column (kept for backward compatibility)
    - active_angle_col: optional; if None and a 'q_deg' column exists, it will be used as active.
    """
    # Peek header to determine availability
    header = self._load_csv_columns(path, [time_col]).keys()
    header = list(header)  # not actually header; adjust by reading header separately below
    # Re-read true header line explicitly
    with open(os.path.abspath(path), "r", encoding="utf-8") as f:
      header = f.readline().strip().split(",")

    required = [time_col]
    passive_present = angle_col in header
    if passive_present:
      required.append(angle_col)

    if active_angle_col is None:
      active_angle_col = "q_deg" if "q_deg" in header else None
    active_present = (active_angle_col is not None) and (active_angle_col in header)
    if active_present:
      required.append(active_angle_col)

    cols = self._load_csv_columns(path, required)
    t = cols[time_col]
    t0 = t[0]
    out = {"t": t - t0, "passive_deg": None, "active_deg": None, "angle_deg": None}
    if passive_present:
      out["passive_deg"] = cols[angle_col]
      out["angle_deg"] = out["passive_deg"]
    if active_present:
      out["active_deg"] = cols[active_angle_col]
    self.exp_angles = out
    return self

  def load_experiment_force_csv(
      self,
      path: str,
      time_col: str = "time_s",
      force_col: str = "fz_lp_N",
  ):
    cols = self._load_csv_columns(path, [time_col, force_col])
    t = cols[time_col]
    self.exp_force = {"t": t - t[0], "fz_N": -cols[force_col]}
    return self

  @staticmethod
  def _interp_to_grid(t_src: np.ndarray, y_src: np.ndarray, t_grid: np.ndarray) -> np.ndarray:
    """
    Linear interpolation onto t_grid.
    Values outside [min(t_src), max(t_src)] become NaN so RMS ignores them cleanly.
    """
    y = np.interp(t_grid, t_src, y_src)
    out = y.astype(float)
    out[t_grid < t_src.min()] = np.nan
    out[t_grid > t_src.max()] = np.nan
    return out

  def align_experiment_to_sim_time(self):
    """
    Create aligned arrays on the simulation time grid:
      aligned['angle_deg'] = {'t': t_sim, 'exp': ..., 'sim': ...}
      aligned['fz_N']      = {'t': t_sim, 'exp': ..., 'sim': ...}
    """
    t_sim = np.asarray(self.t, dtype=float)
    if t_sim.size == 0:
      raise RuntimeError("No simulation data. Run the simulation first.")

    # Simulation signals (pick first paddle joint + first force sensor by default)
    if not self.paddle_jnt_ids:
      raise RuntimeError("No paddle joints found to compare.")
    if not self.force_sensor_ids:
      raise RuntimeError("No force sensors found to compare.")

    motor_jnt_id = self.motor_jnt_ids[0]
    paddle_jnt_id = self.paddle_jnt_ids[0]
    sensor_id = self.force_sensor_ids[0]

    # Correct per-joint series: do not mix ids/dicts
    sim_motor_angle_deg = np.degrees(np.asarray(self.ang_motor[motor_jnt_id], dtype=float))
    # Incorrect old line that addressed paddle with motor id; keep commented to clarify passive-only bug.
    # sim_angle_deg = np.degrees(np.asarray(self.ang_paddle[motor_jnt_id], dtype=float))
    sim_paddle_angle_deg = np.degrees(np.asarray(self.ang_paddle[paddle_jnt_id], dtype=float))
    sim_fz = np.asarray(self.force[sensor_id]["z"], dtype=float)

    # Angles exp -> sim grid (both passive and active if available)
    if self.exp_angles is not None:
      t_exp = self.exp_angles["t"]
      exp_passive_on_sim = None
      exp_active_on_sim = None
      if self.exp_angles.get("passive_deg") is not None:
        exp_passive_on_sim = self._interp_to_grid(t_exp, self.exp_angles["passive_deg"], t_sim)
      if self.exp_angles.get("active_deg") is not None:
        exp_active_on_sim = self._interp_to_grid(t_exp, self.exp_angles["active_deg"], t_sim)
    else:
      exp_passive_on_sim = None
      exp_active_on_sim = None

    # Store aligned packs
    self.aligned["paddle_angle_deg"] = {"t": t_sim, "exp": exp_passive_on_sim, "sim": sim_paddle_angle_deg}
    self.aligned["motor_angle_deg"] = {"t": t_sim, "exp": exp_active_on_sim, "sim": sim_motor_angle_deg}
    # Backward compatibility with existing plotting flow for passive paddle angle
    self.aligned["angle_deg"] = self.aligned["paddle_angle_deg"]

    # Force exp -> sim grid
    if self.exp_force is not None:
      exp_fz_on_sim = self._interp_to_grid(self.exp_force["t"], self.exp_force["fz_N"], t_sim)
      self.aligned["fz_N"] = {"t": t_sim, "exp": exp_fz_on_sim, "sim": sim_fz}
    else:
      self.aligned["fz_N"] = {"t": t_sim, "exp": None, "sim": sim_fz}

    return self

  # ============================ RMS + plotting ============================

  @staticmethod
  def rms_error(t: np.ndarray, y_sim: np.ndarray, y_exp: np.ndarray, t0: float = 0.0, duration: float | None = None) -> float:
    """
    RMS(sim-exp) over a window:
      start at t >= t0
      optionally limit to t <= t0+duration
    Ignores NaNs in exp or sim.
    """
    t = np.asarray(t, dtype=float)
    y_sim = np.asarray(y_sim, dtype=float)
    y_exp = np.asarray(y_exp, dtype=float)

    mask = t >= t0
    if duration is not None:
      mask &= (t <= (t0 + duration))

    # valid finite points
    mask &= np.isfinite(y_sim) & np.isfinite(y_exp)

    if not np.any(mask):
      return float("nan")

    diff = y_sim[mask] - y_exp[mask]
    return float(np.sqrt(np.mean(diff * diff)))

  def plot_compare_angles(self, t0: float = 0.0, duration: float | None = None):
    if "angle_deg" not in self.aligned:
      raise RuntimeError("Call align_experiment_to_sim_time() first.")

    pack = self.aligned["angle_deg"]
    t = pack["t"]
    sim = pack["sim"]
    exp = pack["exp"]

    # Plot raw passive angles (no mean subtraction)
    sim_plot = np.array(sim, dtype=float)
    exp_plot = None if exp is None else np.array(exp, dtype=float)

    plt.figure()
    plt.plot(t, sim_plot, label="sim paddle angle [deg]")
    if exp_plot is not None:
      plt.plot(t, exp_plot, label="exp paddle angle [deg]")
      rms = self.rms_error(t, sim, exp, t0=t0, duration=duration)
      plt.title(f"Angle: sim vs exp (RMS={rms:.4g} deg)")
    else:
      plt.title("Angle: simulation only")

    plt.xlabel("Time [s]")
    plt.ylabel("Angle [deg]")
    plt.grid(True)
    plt.legend()
    plt.show()

  def plot_compare_active_angles(self, t0: float = 0.0, duration: float | None = None):
    if "motor_angle_deg" not in self.aligned:
      raise RuntimeError("Call align_experiment_to_sim_time() first.")

    pack = self.aligned["motor_angle_deg"]
    t = pack["t"]
    sim = pack["sim"]
    exp = pack["exp"]

    # Subtract mean offset from experimental active angle over the selected window
    sim_plot = np.array(sim, dtype=float)
    exp_plot = None if exp is None else np.array(exp, dtype=float)
    if exp_plot is not None:
      mask = (t >= t0)
      if duration is not None:
        mask &= (t <= (t0 + duration))
      mask &= np.isfinite(exp_plot)
      if np.any(mask):
        exp_plot = exp_plot - float(np.mean(exp_plot[mask]))

    plt.figure()
    plt.plot(t, sim_plot, label="sim motor angle [deg]")
    if exp_plot is not None:
      plt.plot(t, exp_plot, label="exp motor angle [deg]")
      rms = self.rms_error(t, sim, exp, t0=t0, duration=duration)
      plt.title(f"Motor angle: sim vs exp (RMS={rms:.4g} deg)")
    else:
      plt.title("Motor angle: simulation only")

    plt.xlabel("Time [s]")
    plt.ylabel("Angle [deg]")
    plt.grid(True)
    plt.legend()
    plt.show()

  def plot_compare_force(self, t0: float = 0.0, duration: float | None = None):
    if "fz_N" not in self.aligned:
      raise RuntimeError("Call align_experiment_to_sim_time() first.")

    pack = self.aligned["fz_N"]
    t = pack["t"]
    sim = pack["sim"]
    exp = pack["exp"]

    # Do NOT subtract mean for force here; plot raw values
    sim_plot = np.array(sim, dtype=float)
    exp_plot = None if exp is None else np.array(exp, dtype=float)

    plt.figure()
    plt.plot(t, sim_plot, label="sim fz [N]")
    if exp_plot is not None:
      plt.plot(t, exp_plot, label="exp fz [N]")
      rms = self.rms_error(t, sim, exp, t0=t0, duration=duration)
      plt.title(f"Force: sim vs exp (RMS={rms:.4g} N)")
    else:
      plt.title("Force: simulation only")

    plt.xlabel("Time [s]")
    plt.ylabel("Force [N]")
    plt.grid(True)
    plt.legend()
    plt.show()

  @staticmethod
  def _slice_by_offsets(sim: np.ndarray, exp: np.ndarray, sim_offset: int, exp_offset: int, length: int | None):
    """
    Returns (sim_seg, exp_seg) aligned by index offsets.
    If length is None, uses the maximum possible overlap.
    """
    sim = np.asarray(sim, dtype=float)
    exp = np.asarray(exp, dtype=float)

    if sim_offset < 0 or exp_offset < 0:
      raise ValueError("Offsets must be >= 0")

    sim_avail = sim.size - sim_offset
    exp_avail = exp.size - exp_offset
    if sim_avail <= 0 or exp_avail <= 0:
      raise ValueError("Offset beyond signal length")

    max_len = min(sim_avail, exp_avail)
    L = max_len if length is None else min(length, max_len)
    if L <= 0:
      raise ValueError("Non-positive comparison length")

    sim_seg = sim[sim_offset:sim_offset + L]
    exp_seg = exp[exp_offset:exp_offset + L]
    return sim_seg, exp_seg

  @staticmethod
  def rms_error_by_index(sim: np.ndarray, exp: np.ndarray, sim_offset: int = 0, exp_offset: int = 0, length: int | None = None) -> float:
    sim_seg, exp_seg = MuJoCoRobotSim._slice_by_offsets(sim, exp, sim_offset, exp_offset, length)
    mask = np.isfinite(sim_seg) & np.isfinite(exp_seg)
    if not np.any(mask):
      return float("nan")
    diff = sim_seg[mask] - exp_seg[mask]
    return float(np.sqrt(np.mean(diff * diff)))

  def plot_compare_angles_by_index(self, sim_offset: int = 0, exp_offset: int = 0, length: int | None = None):
    if "angle_deg" not in self.aligned:
      raise RuntimeError("Call align_experiment_to_sim_time() first.")
    pack = self.aligned["angle_deg"]
    if pack["exp"] is None:
      raise RuntimeError("No experiment angle data loaded.")

    sim = np.asarray(pack["sim"], dtype=float)
    exp = np.asarray(pack["exp"], dtype=float)

    sim_seg, exp_seg = self._slice_by_offsets(sim, exp, sim_offset, exp_offset, length)
    # Plot raw passive segments (no mean subtraction)
    sim_seg_plot = np.array(sim_seg, dtype=float)
    exp_seg_plot = np.array(exp_seg, dtype=float)
    x = np.arange(sim_seg.size)

    rms = self.rms_error_by_index(sim, exp, sim_offset, exp_offset, length)

    plt.figure()
    plt.plot(x, sim_seg_plot, label="sim paddle angle [deg]")
    plt.plot(x, exp_seg_plot, label="exp paddle angle [deg]")
    plt.title(f"Angle (index-aligned): RMS={rms:.4g} deg | sim_off={sim_offset} exp_off={exp_offset} N={sim_seg.size}")
    plt.xlabel("Sample index (comparison window)")
    plt.ylabel("Angle [deg]")
    plt.grid(True)
    plt.legend()
    # plt.show()

  def plot_compare_active_angles_by_index(self, sim_offset: int = 0, exp_offset: int = 0, length: int | None = None):
    if "motor_angle_deg" not in self.aligned:
      raise RuntimeError("Call align_experiment_to_sim_time() first.")
    pack = self.aligned["motor_angle_deg"]
    if pack["exp"] is None:
      raise RuntimeError("No experiment active angle data loaded.")

    sim = np.asarray(pack["sim"], dtype=float)
    exp = np.asarray(pack["exp"], dtype=float)

    sim_seg, exp_seg = self._slice_by_offsets(sim, exp, sim_offset, exp_offset, length)
    # Subtract mean from experimental active segment for plotting only
    sim_seg_plot = np.array(sim_seg, dtype=float)
    exp_seg_plot = np.array(exp_seg, dtype=float)
    m = np.isfinite(exp_seg_plot)
    if np.any(m):
      exp_seg_plot = exp_seg_plot - float(np.mean(exp_seg_plot[m]))
    x = np.arange(sim_seg.size)

    rms = self.rms_error_by_index(sim, exp, sim_offset, exp_offset, length)

    plt.figure()
    plt.plot(x, sim_seg_plot, label="sim motor angle [deg]")
    plt.plot(x, exp_seg_plot, label="exp motor angle [deg]")
    plt.title(f"Motor angle (index-aligned): RMS={rms:.4g} deg | sim_off={sim_offset} exp_off={exp_offset} N={sim_seg.size}")
    plt.xlabel("Sample index (comparison window)")
    plt.ylabel("Angle [deg]")
    plt.grid(True)
    plt.legend()
    # plt.show()

  def plot_compare_force_by_index(self, sim_offset: int = 0, exp_offset: int = 0, length: int | None = None):
    if "fz_N" not in self.aligned:
      raise RuntimeError("Call align_experiment_to_sim_time() first.")
    pack = self.aligned["fz_N"]
    if pack["exp"] is None:
      raise RuntimeError("No experiment force data loaded.")

    sim = np.asarray(pack["sim"], dtype=float)
    exp = np.asarray(pack["exp"], dtype=float)

    sim_seg, exp_seg = self._slice_by_offsets(sim, exp, sim_offset, exp_offset, length)
    # Plot raw force segments; no mean subtraction
    sim_seg_plot = np.array(sim_seg, dtype=float)
    exp_seg_plot = np.array(exp_seg, dtype=float)
    x = np.arange(sim_seg.size)

    rms = self.rms_error_by_index(sim, exp, sim_offset, exp_offset, length)

    plt.figure()
    plt.plot(x, sim_seg_plot, label="sim fz [N]")
    plt.plot(x, exp_seg_plot, label="exp fz [N]")
    plt.title(f"Force (index-aligned): RMS={rms:.4g} N | sim_off={sim_offset} exp_off={exp_offset} N={sim_seg.size}")
    plt.xlabel("Sample index (comparison window)")
    plt.ylabel("Force [N]")
    plt.grid(True)
    plt.legend()
    plt.show()

def main():
  script_dir = os.path.dirname(os.path.abspath(__file__))
  model_path = os.path.abspath(os.path.join(script_dir, "../../../models/xml/test_force.xml"))
  csv_dir =  os.path.abspath(os.path.join(script_dir, "../../../exp/20260116_111142_A30_f1.0"))
                            
  sim = MuJoCoRobotSim(
    model_path=model_path,
    keyframe_name="init_pose",
    step=0.001,
    sim_time=10.0,
    amp=math.pi/6,
    freq=1.0,
    passive_viewer=False,
    force_sensor_substr="force",
  )
  
  sim.run()
  # sim.plot_basic()

  sim.load_experiment_angles_csv(
    path=os.path.join(csv_dir, "A030deg_f1000mHz_angle.csv"),
    time_col="time_shifted",
    angle_col="q_deg",           # passive (paddle) angle
    active_angle_col="theta12_deg",          # active (motor) angle
  )
  sim.load_experiment_force_csv(
    path=os.path.join(csv_dir, "A030deg_f1000mHz_force.csv"),
    time_col="time_s",
    force_col="fz_lp_N",
  )
  sim.align_experiment_to_sim_time()

  # Plot with arbitrary RMS window start
  # Example: start comparing at t0=5.0s, for 2 seconds:
  # t0 = 5.0
  # duration = 2.0
  # sim.plot_compare_angles(t0=t0, duration=duration)
  # sim.plot_compare_force(t0=t0, duration=duration)
  sim_offset = 2050
  exp_offset = 960
  length = 4000
  sim.plot_compare_angles_by_index(sim_offset=sim_offset, exp_offset=exp_offset, length=length)
  # Also compare active (motor) joint angles
  try:
    sim.plot_compare_active_angles_by_index(sim_offset=sim_offset, exp_offset=exp_offset, length=length)
  except RuntimeError:
    # If active exp data not available, skip without breaking plotting flow
    pass
  
  sim.plot_compare_force_by_index(sim_offset=sim_offset, exp_offset=exp_offset, length=length)
  rms_angle = sim.rms_error_by_index(sim.aligned["angle_deg"]["sim"], sim.aligned["angle_deg"]["exp"],
                                   sim_offset=sim_offset, exp_offset=exp_offset, length=length)
  print("RMS angle:", rms_angle)

if __name__ == "__main__":
  main()
