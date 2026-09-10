# mobile_flight

ROS 2 (Humble) offboard velocity control stack for PX4-based UAVs.  
Supports both **Gazebo SITL simulation** and **real hardware** via Micro XRCE-DDS.

---

## Repository Structure

```
mobile_flight/
├── docker/
│   ├── Dockerfile          # PX4 + ROS 2 Humble + Gazebo Harmonic image
│   ├── docker-compose.yml  # Container orchestration
│   └── entrypoint.sh       # Runtime setup (PX4 clone, workspace build)
├── mobile_gazebo/
│   ├── src/
│   │   └── offboard_velocity_control.cpp  # Main control node
│   └── launch/
│       └── sim_uxrce_dds.launch.py        # Simulation launch file
└── mobile_msgs/
    └── srv/
        ├── SetVelocity.srv
        └── Takeoff.srv
```

---

## Packages

| Package | Description |
|---------|-------------|
| `mobile_gazebo` | Main flight control node (`offboard_velocity_control`) |
| `mobile_msgs` | Custom ROS 2 service definitions |

---

## Building the Workspace

### Prerequisites
- Docker + Docker Compose installed on the host
- For real hardware: Pixhawk connected via USB (`/dev/ttyACM0`)

### Inside the Docker container

```bash
# 1. Build and enter the container
cd ~/imav_ws/src/mobile_flight/docker
docker compose up -d
docker exec -it mobile_flight_sim bash

# 2. Build the workspace (first time only — entrypoint does this automatically)
cd ~/imav_ws
colcon build --symlink-install

# 3. Source the overlay
source ~/imav_ws/install/setup.bash
```

---

## Running — Simulation (SITL + Gazebo)

Open **3 terminals** inside the container (`docker exec -it mobile_flight_sim bash` each):

**Terminal 1 — PX4 SITL + Gazebo**
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

**Terminal 2 — Micro XRCE-DDS Agent (UDP for SITL)**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 3 — Offboard control node**
```bash
source ~/imav_ws/install/setup.bash
ros2 run mobile_gazebo offboard_velocity_control
```

Or use the provided launch file (runs SITL + node together):
```bash
ros2 launch mobile_gazebo sim_uxrce_dds.launch.py
```

---

## Running — Real Hardware (Pixhawk via USB)

Open **2 terminals** inside the container:

**Terminal 1 — Micro XRCE-DDS Agent (Serial over USB)**
```bash
# Check the device first on the HOST:  ls /dev/ttyACM*
MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600
```

> **PX4 parameter required:** Set `UXRCE_DDS_CFG` to the USB port (e.g., `Telem 2` or `USB`) in QGroundControl → Parameters.

**Terminal 2 — Offboard control node**
```bash
source ~/imav_ws/install/setup.bash
ros2 run mobile_gazebo offboard_velocity_control
```

---

## `offboard_velocity_control` Node

### Overview

Controls the drone in **PX4 Offboard mode** via velocity setpoints.  
Publishes setpoints at **10 Hz** and exposes ROS 2 services for arming, takeoff, velocity control, landing, and emergency stop.

### State Machine

```
INIT ──► TAKEOFF ──► VELOCITY
  ▲                     │
  └─── land / estop ────┘
```

| State | Behavior |
|-------|----------|
| `INIT` | Publishes zero velocity; waits for commands |
| `TAKEOFF` | Position control to target altitude (NED frame) |
| `VELOCITY` | Velocity control; auto-hovers when duration expires or heartbeat times out |

---

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | Tells PX4 which control dimension is active (position or velocity) |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | Position or velocity setpoint sent to PX4 at 10 Hz |
| `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | MAVLink-style commands (arm, disarm, mode switch, land) |

---

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | Current yaw used for body→NED frame rotation |
| `/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` | Arming state and navigation state monitoring |
| `/fmu/out/timesync_status` | `px4_msgs/TimesyncStatus` | XRCE-DDS link heartbeat (warns if silent >2 s) |
| `/fmu/out/battery_status` | `px4_msgs/BatteryStatus` | Battery voltage and remaining capacity |
| `/fmu/out/estimator_status` | `px4_msgs/EstimatorStatus` | Checks if horizontal/vertical position and velocity are valid |
| `/fmu/out/vehicle_gps_position` | `px4_msgs/SensorGps` | GPS fix type and satellite count (informational) |

---

## Services

### `~/arm` — `std_srvs/Trigger`
Arms the drone and switches to Offboard mode.

**Pre-flight checks:**
- ESTOP must **not** be active
- Estimator must report valid position + velocity

```bash
ros2 service call /offboard_velocity_control/arm std_srvs/srv/Trigger
```

---

### `~/takeoff` — `mobile_msgs/Takeoff`
Arms, switches to Offboard mode, and climbs to the requested altitude.

| Field | Type | Description |
|-------|------|-------------|
| `altitude` | `float32` | Target altitude in **metres** (positive = up) |

**Pre-flight checks:** ESTOP inactive, estimator valid, battery warning at <15%.  
**Limit:** Clamped to max 50 m (`MAX_ALTITUDE`).

```bash
ros2 service call /offboard_velocity_control/takeoff \
  mobile_msgs/srv/Takeoff "altitude: 5.0"
```

---

### `~/set_velocity` — `mobile_msgs/SetVelocity`
Commands a velocity for an optional duration.

| Field | Type | Description |
|-------|------|-------------|
| `vx` | `float32` | X velocity (m/s) — forward in `local`, forward in `body` |
| `vy` | `float32` | Y velocity (m/s) — left in `local`, left in `body` |
| `vz` | `float32` | Z velocity (m/s) — **NED: negative = up** |
| `yaw` | `float32` | Target yaw (rad) |
| `duration` | `float32` | Duration in seconds (`0` = indefinite) |
| `frame_id` | `string` | `"local"` (NED) or `"body"` (drone-relative) |
| `auto_arm` | `bool` | If `true`, arms and switches to Offboard automatically |

**Limits:** `vx`, `vy` clamped to ±5 m/s · `vz` clamped to ±2 m/s.  
When `frame_id = "body"`, velocities are rotated to NED using the current yaw heading.

```bash
# Fly forward 1 m/s for 3 seconds (body frame)
ros2 service call /offboard_velocity_control/set_velocity \
  mobile_msgs/srv/SetVelocity \
  "{vx: 1.0, vy: 0.0, vz: 0.0, yaw: 0.0, duration: 3.0, frame_id: 'body', auto_arm: false}"
```

---

### `~/land` — `std_srvs/Trigger`
Commands PX4 `NAV_LAND` and resets the node to `INIT` state.

```bash
ros2 service call /offboard_velocity_control/land std_srvs/srv/Trigger
```

---

### `~/estop` — `std_srvs/Trigger`
**Emergency stop.** Commands immediate landing and locks all further commands.

> ⚠️ Once triggered, ESTOP can only be cleared by restarting the node.

```bash
ros2 service call /offboard_velocity_control/estop std_srvs/srv/Trigger
```

---

## Safety Features

| Feature | Detail |
|---------|--------|
| **Estimator guard** | ARM and TAKEOFF rejected if EKF2 has not fused horizontal position + velocity |
| **Battery warning** | Warning logged at takeoff if remaining capacity <15% |
| **Velocity clamping** | `vx`, `vy` ≤ 5 m/s · `vz` ≤ 2 m/s |
| **Altitude limit** | Takeoff target clamped to 50 m AGL |
| **Heartbeat failsafe** | Auto-hover if no `set_velocity` call received for >5 s |
| **XRCE-DDS watchdog** | Warning logged every 1 s if timesync silent for >2 s |
| **ESTOP lock** | Emergency stop disables all commands until node restart |

---

## Custom Service Definitions

### `mobile_msgs/Takeoff`
```
float32 altitude   # metres, positive = up
---
bool    success
string  message
```

### `mobile_msgs/SetVelocity`
```
float32 vx         # m/s (clamped ±5.0)
float32 vy         # m/s (clamped ±5.0)
float32 vz         # m/s NED (clamped ±2.0)
float32 yaw        # rad
float32 duration   # seconds (0 = indefinite)
string  frame_id   # 'local' or 'body'
bool    auto_arm
---
bool    success
string  message
```

---

## Dependencies

| Dependency | Version |
|------------|---------|
| ROS 2 | Humble |
| PX4-Autopilot | latest (`main`) |
| px4_msgs | latest (`main`) |
| px4_ros_com | latest (`main`) |
| Gazebo Harmonic | `gz-harmonic` |
| Micro XRCE-DDS Agent | v2.4.3 |
