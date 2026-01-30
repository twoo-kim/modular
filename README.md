# Modular Robot Simulation (MuJoCo + ROS2)
Simulation of modular robots with one active motor joint and one passive paddle joint per module. Run via pure Python scripts or as ROS2 C++ nodes. Includes linear time-periodic (LTP) analysis and harmonic transfer function (HTF) tools.

## Overview
- MuJoCo-based dynamics with optional OpenGL viewer.
- Python scripts for quick experiments and plots (no ROS needed).
- ROS2 C++ simulator + per-module controllers for multi-agent setups.
- Offline C++ runner computes LTV/HTF for periodic behavior analysis.

## Quick Start

Follow these steps depending on whether you want Python-only runs or the full ROS2 C++ stack.

### A) Python-only (no ROS)
1) Install Python packages
   - `pip install mujoco matplotlib numpy`

2) Verify MuJoCo Python works
   - `python -c "import mujoco; print(mujoco.__version__)"`

3) Run examples
   - Thrust sweep: `python src/controller/scripts/test_force.py`
   - Single module time series: `python src/controller/scripts/test_module.py`
   - Compare sim vs experiment: update CSV path and run `src/controller/scripts/test_experiment.py`

Notes
- These scripts load XML from `models/xml/` directly; no ROS is required.

### B) ROS2 C++ Simulator
Prerequisites
- Installed ROS2 (any supported distro), C++ toolchain, and `colcon`.
- MuJoCo binaries and GLFW development headers.

1) Install MuJoCo binaries
   - Download from: https://github.com/google-deepmind/mujoco/releases (e.g., `mujoco-3.3.7`).
   - Place at: `$HOME/.mujoco/mujoco-3.3.7/`.
   - Add to your shell init (e.g., `~/.bashrc`):
     
     ```
     export MUJOCO_PATH=$HOME/.mujoco/mujoco-3.3.7
     export LD_LIBRARY_PATH=$MUJOCO_PATH/lib:$MUJOCO_PATH/bin:$LD_LIBRARY_PATH
     ```
   - Apply changes: `source ~/.bashrc`.

2) Install GLFW3 (viewer)
   - Ubuntu/Debian: `sudo apt-get install -y libglfw3-dev`

3) Build the packages
   - From your ROS2 workspace:
     ```
     colcon build --packages-up-to modular
     source install/setup.bash
     ```

4) Configure the simulator
   - Edit `src/controller/config/modular_params.yaml` to set:
     - `N` (number of modules), `SIM_TIME`, `TIME_STEP`, `isFixed`, `visualize`.
   - The simulator generates a colony XML under `models/xml/colony/` at runtime.

5) Run
   - `ros2 launch modular modular_launch.py`
   - This launches N controller nodes and one simulator node. The simulator publishes `/pose_topic`; each controller publishes to its own `<prefix>module/control` topic (e.g., `r1_module/control`).

Troubleshooting
- If the viewer fails, set `visualize: false` in the YAML or install/configure OpenGL/X forwarding.
- If MuJoCo libraries aren’t found, re-check `LD_LIBRARY_PATH` and `MUJOCO_PATH`.
- The CMake file uses a `MUJOCO_PATH` use absolute paths or rely on system include/lib paths.

### C) Dynamics Simulation + LTP/HTF (offline C++)
Use the standalone C++ runners to step MuJoCo, collect LTV (A/B) along a periodic trajectory, and build the harmonic transfer function.

Prerequisites
- Same as B) (MuJoCo + GLFW). Also ensure Python dev headers are available (CMake links `Python3::Python` and `Python3::NumPy`). On Ubuntu/Debian: `sudo apt-get install -y python3-dev python3-numpy`.

1) Build
   - From your ROS2 workspace:
     ```
     colcon build --packages-up-to modular
     source install/setup.bash
     ```

2) Configure analysis
   - Edit `src/controller/config/modular_params.yaml`:
     - Time: set `SIM_TIME` long enough to cover several periods at your frequency (e.g., ≥ 8–10 periods).
     - LTP/HTF: set `harmonics` (e.g., 20) and `sampling_f` (Hz) for Fourier blocks.
     - Optional: `visualize: false` for faster, headless runs.
   - IMPORTANT: The sample runners hard-code `config_path` in C++:
     - `src/controller/src/dynamics_simulation.cpp`
     - `src/controller/src/dynamics_simulation_multi.cpp`
     Update `config_path` to your local `modular_params.yaml` path, then rebuild.

3) Run
   - Single-case:\
      `ros2 run modular colony_sim`
   - Phase sweep:\
   `ros2 run modular colony_sim_multi --phases=0,30,60,90,120,150,180`
    
    - If argument parsing via `ros2 run` is problematic, run the binary directly:
       `install/modular/lib/modular/colony_sim_multi --phases=0,30,60,90`

4) Inspect outputs
   - Joint angles over time for modules.
   - Periodicity check: norms of A(2s)–A(t), B(2s)–B(t) over one period.
   - HTF magnitude vs harmonic order for selected joints (e.g., module 1 passive).

Tips
- Ensure the fundamental frequency used in the runner matches your intended study; the examples set frequency and amplitude inside the C++ files via `setFrequency()`/`setAmplitude()`.
- For stable HTF estimates, the routine ignores the initial transient (≈ first 2 s).

## Repository Structure
- `models/`:
  - `xml/modular.xml`: single-module robot definition (joints, sensors, actuator).
  - `xml/environment.xml`, `xml/asset.xml`: visuals/materials.
  - `xml/colony/`: auto-generated multi-module colony XMLs.
  - `urdf/`: URDF variants (not used by the runtime sim).
- `modular_msgs/`: ROS2 message package (`ControlMsg.msg`, `StateMsg.msg`).
- `src/controller/`:
  - `config/modular_params.yaml`: central configuration (N, dt, gains, fluids, LTP, phases).
  - `include/`: headers for config, modules, LTV, simulator, viewer, utils, XML generation.
  - `src/simulation/*.cpp`: implementations for viewer, ROS simulator, dynamics+LTP.
  - `src/*.cpp`: main entrypoints (`sim_node`, `ctrl_node`, `colony_sim`, `colony_sim_multi`).
  - `launch/modular_launch.py`: spawns N controllers + one simulator.
  - `scripts/`: Python demos (`test_force.py`, `test_module.py`, `test_experiment.py`).
- `exp/`: experimental CSVs and PDFs for validation/alignment.

## Workflows
### Python (no ROS)
1) Install: `pip install mujoco matplotlib numpy`  
2) Examples:
- Sweep thrust vs angle/frequency: `python src/controller/scripts/test_force.py`
- Single module time series: `python src/controller/scripts/test_module.py`
- Compare sim vs experiment (RMS, overlays): `python src/controller/scripts/test_experiment.py`

### ROS2 C++
1) Install MuJoCo binaries and GLFW (`libglfw3-dev`). Ensure `LD_LIBRARY_PATH` includes MuJoCo libs.  
2) Build in your ROS2 workspace: `colcon build --packages-up-to modular modular_msgs`  
3) Launch: `ros2 launch modular modular_launch.py`  
4) Tune `src/controller/config/modular_params.yaml` (e.g., `N`, `phase_gap`, `isFixed`).

### Offline LTP/HTF (C++)
Build then run:
- `modular/colony_sim`: single-case plots (joints, thrust, periodicity, HTF magnitude).
- `modular/colony_sim_multi`: phase-sweep overlays (thrust, passive angle, HTF).  
Adjust analysis in `modular_params.yaml` (`harmonics`, `sampling_f`, `SIM_TIME`).

## Configuration (YAML)
File: `src/controller/config/modular_params.yaml`
- Geometry: `N`, `RADIUS`, `FACE_RADIAL`, `YAW_OFFSET_DEG`, `ADD_DUMMY_MASS`.
- Time: `SIM_TIME`, `TIME_STEP`, `isEuler` (Euler vs RK4), `isFixed` (free base vs fixed).
- Control: `is_activate` (per-module enable), `P_gain`, `D_gain`.
- Fluid: `Cn`, `rho`, `viscosity`.
- LTP/HTF: `harmonics`, `sampling_f`, `phase_gap` (per-module phase offsets in radians).

## ROS Interfaces
- Messages:
  - `modular_msgs/msg/ControlMsg`: `index` (1-based module ID), `ctrl` (control input), `phi` (internal phase).
  - `modular_msgs/msg/StateMsg`: `index[]`, `joint_motor[]`, `joint_paddle[]`, `force[]`, `pose[]`, `twist[]`, `phase[]`.
- Topics:
  - Simulator publishes `/pose_topic`.
  - Controllers publish `<prefix>module/control` (e.g., `r1_module/control`).
- Launch: `launch/modular_launch.py` reads `N` from YAML, starts N controller nodes + one simulator.

## Key Files
- Config: `src/controller/config/modular_params.yaml` (main knobs most users change).
- XML Gen: `src/controller/include/utils/xml_utils.hpp` (creates `models/xml/colony/test_<N>_circle.xml`).
- ROS Sim: `src/controller/src/simulation/mujoco_simulator.cpp` (subscribes control, steps sim, publishes state).
- Dynamics + LTP: `src/controller/src/simulation/dynamics_simulator.cpp` (linearization, HTF building, plotting).
- Experiments: `src/controller/scripts/test_experiment.py` (CSV load, alignment, RMS, plotting).

## Design Notes
- Control: per-module PD maps desired position-style input to actuator effort; enable via `is_activate`.
- Phase: `phase_gap[]` sets module offsets (used in offline runners; extendable to controllers).
- Linearization: LTV (A/B) captured each step with `mjd_transitionFD`; HTF assembled from Fourier blocks.
- Fluid: paddle uses `fluidshape`/`fluidcoef`; global density/viscosity set via YAML → XML.

## Known Pitfalls
- Absolute paths: some offline runners hard-code config path; switch to a relative path or pass via CLI.
- MuJoCo path in CMake: avoid `~` in `MUJOCO_PATH`; use absolute path or system include/lib.
- Viewer: set `visualize: true` only with a working OpenGL/X setup. Headless runs fine otherwise.
- LTP windows: HTF uses post-transient data (≈2 s onwards). Ensure enough `SIM_TIME` for your frequency.

 
