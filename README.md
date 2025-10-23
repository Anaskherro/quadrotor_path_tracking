# quadrotor_path_tracking

Path tracking for quadrotor UAVs using **PID** and **MPC** controllers.  
This repo takes planned paths (e.g. from `quadrotor_path_planning/paths`) and executes them with MAVROS + Gazebo.

---

## Step-by-step setup

### 1. Prepare environment
- Ubuntu 20.04 + ROS Noetic  
- Install dependencies:
  ```bash
  sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
  ```
- Clone the repo inside your catkin_ws:
  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/Intelligent-Quads/iq_sim.git
  git clone https://github.com/Intelligent-Quads/iq_gnc.git
  git clone https://github.com/Anaskherro/quadrotor_path_tracking.git
  cd ~/catkin_ws && catkin build
  ```

### 2. Run autopilot (SITL)
- Use ArduPilot SITL (from planning setup).
- Launch Iris or X500 in Gazebo:
  ```bash
  roslaunch iris_path_tracking apm.launch
  # or
  roslaunch x500_path_tracking apm.launch
  ```

---

## 3. PID Tracking
1. Place the planned path file in `paths/path.txt`.  
2. Launch simulation with MAVROS:
   ```bash
   roslaunch iris_path_tracking lidar.launch
   ```
3. Run PID controller:
   ```bash
   python3 drone_control/command.py
   python3 drone_control/dronepath.py   # visualize trajectory
   ```

---

## 4. MPC Tracking
1. Install **mrs_uav_system**:
   ```bash
   sudo apt install ros-noetic-mrs-uav-system-full
   ```
2. Launch MPC simulation:
   ```bash
   roslaunch mrs_uav_gazebo_simulation tmux/one_drone.launch
   ```
3. Edit `config/example_params.yaml` to load trajectory from `paths/path.txt`.  
4. Start trajectory tracking:
   ```bash
   rosservice call /uav1/control_manager/start_trajectory_tracking
   ```

---

## Output
- Rviz shows reference (blue) vs tracked path (orange/green).  
- Gazebo simulates quadrotor following trajectory.  
- MPC usually yields smoother & more predictive control than PID.

---

## Repo structure

```
quadrotor_path_tracking/
├─ paths/                  # Input trajectories
├─ path_transformations/   # Path smoothing, resampling
├─ drone_control/          # PID/MPC controllers
├─ iris_path_tracking/     # Iris-specific configs
├─ x500_path_tracking/     # X500-specific configs
└─ README.md
```

---

## License
MIT (see LICENSE)
