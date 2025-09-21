# quadrotor_path_tracking

Path tracking for quadrotor UAVs using PID and MPC controllers.  
Uses planned paths (from `quadrotor_path_planning`) and implements controllers to follow waypoints, while performing necessary coordinate transforms and interpolation.

---

## Overview

This repository provides multiple path‐tracking modules, organized per controller type and per vehicle platform. It supports:

- PID controller tracking for Iris, X500, etc.  
- MPC controller tracking when available.  
- Path transformations (resampling, smoothing, interpolation).  
- Multiple tracking implementations under different vehicle models.

---

## Repository structure

```
quadrotor_path_tracking/
├─ paths/                      # Input path files (from path planning)
├─ path_transformations/       # Resampling, smoothing, interpolation routines
├─ drone_control/              # Controller‐specific code (PID / MPC)
├─ iris_path_tracking/         # Tracking tuned for Iris model
├─ x500_path_tracking/         # Tracking tuned for X500 vehicle
└─ README.md
```

---

## Requirements

- Python 3.8+  
- `numpy`, `scipy`, `matplotlib`  
- MPC dependencies: e.g. `cvxpy`  
- ROS/MAVROS (if used in simulation or real deployment)  

---

## Usage

- Place a path file from the planning repo into `paths/`.  
- If necessary, transform or smooth the path via `path_transformations/`.  
- Select a controller:  
  - `drone_control/` for generic quadrotor.  
  - `iris_path_tracking/` for Iris.  
  - `x500_path_tracking/` for X500.  
- Run the appropriate controller script.

Example (PID):
```python
from drone_control.pid_controller import PIDController
controller = PIDController(vehicle_params, path, dt)
controller.run()
```

---

## Configuration variables

Set in code or config modules:

- Vehicle parameters: mass, inertia, max tilt, max thrust  
- Control gains: `KP_pos`, `KI_pos`, `KD_pos`  
- MPC: horizon length, cost weights, constraints  
- Path: smoothing factor, resampling distance  

---

## Best Practices & Tips

- Ensure path is feasible (curvature, acceleration).  
- Start with PID gains before trying MPC.  
- Visualize in simulation (Gazebo or similar) before real deployment.  
- Use smaller time steps for accurate tracking.  

---

## License

MIT License (see `LICENSE`).
