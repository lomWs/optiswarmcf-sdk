# OptiSwarmCF

A modular framework for controlling **Crazyflie swarms** using a **ROS2 backend** and a clean **Python SDK**.

---

## 🧠 Overview

This project is split into two main components:

### 1. Backend (ROS2 / colcon workspace)

The backend handles all low-level communication with:

* **OptiTrack (via VRPN)**
* **Crazyflie drones (via Crazyradio)**

#### Components

* **VRPN Client**

  * Connects to OptiTrack/Motive
  * Publishes raw ROS topics

* **`mocap_bridge_ros2`**

  * Normalizes VRPN topics
  * Outputs canonical topics:

    ```
    /mocap/<drone_id>/pose   (geometry_msgs/PoseStamped)
    ```

* **`cf_bridge`**

  * Interfaces with Crazyflie hardware (via `cflib`)
  * Subscribes to mocap topics and sends position updates
  * Exposes control API per drone:

    ```
    /<ns>/cmd_pos
    /<ns>/cmd_pos_relative
    /<ns>/takeoff
    /<ns>/land
    /<ns>/ekf_reset
    /<ns>/diag
    ```

---

### 2. SDK (Python package)

A lightweight Python SDK that provides clean abstractions:

* `OptiTrack` → reads mocap data
* `CrazyflieAgent` → sends commands
* `Swarm` → multi-agent abstraction
* `RosContext` → ROS lifecycle management

👉 The user writes their own control loop using these objects.

---

## 📁 Project Structure

```
optiswarmcf/
├── backend_ros2/      # ROS2 workspace
│   └── src/
│       ├── mocap_bridge_ros2/
│       └── cf_bridge/
├── scripts/           #Scripts .sh
├── sdk/               # Python SDK (pip package)
│   └── src/optiswarmcf/
├── examples/          # Example controllers
└── README.md
```

---

## ⚙️ Requirements

Install ROS2 tools:

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

Initialize rosdep:

```bash
sudo rosdep init
rosdep update
```

Install Crazyflie Python library:

```bash
python3 -m pip install --user cflib
```

---

## 🏗️ Backend Build

```bash
cd backend_ros2

rosdep install --from-paths src --ignore-src -r -y --skip-keys ament_python

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Verify:

```bash
ros2 pkg list | grep cf_bridge
ros2 pkg list | grep mocap_bridge_ros2
```

---

## ⚙️ Configuration

### mocap_bridge (`mocap.yaml`)

Defines mapping from VRPN topics to canonical topics:

```yaml
frame_id: map
topic_prefix: /mocap
axis_mode: identity

sources:
  cf1:
    topic: "/VRPN_TOPIC_CF1"
    type: "pose_stamped"
```

---

### cf_bridge (`cf_bridge.yaml`)

Defines drones:

```yaml
drones:
  - id: "cf1"
    uri: "radio://0/90/2M/E7E7E7E702"
    ns: "/cf1"
    mocap_topic: "/mocap/cf1/pose"
```

---

## ▶️ Running the Backend

Run each component in a separate terminal.

### 1. VRPN Client

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run vrpn_mocap client_node --ros-args -p server:=192.168.1.10 -p port:=3883
```

---

### 2. Mocap Bridge

```bash
cd backend_ros2
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 launch mocap_bridge_ros2 mocap_bridge.launch.py config_path:=<ABSOLUTE_PATH>/mocap.yaml
```

---

### 3. Crazyflie Bridge

```bash
cd backend_ros2
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 launch cf_bridge cf_bridge.launch.py config_path:=<ABSOLUTE_PATH>/cf_bridge.yaml
```

---

## 🧪 Backend Testing (without SDK)

Publish commands manually:

```bash
ros2 topic pub --once /cf1/cmd_pos geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'map'}, pose: {position: {x: 0, y: 0, z: 0.5}, orientation: {w: 1.0}}}"
```

Call services:

```bash
ros2 service call /cf1/takeoff std_srvs/srv/Trigger "{}"
ros2 service call /cf1/land std_srvs/srv/Trigger "{}"
```

---

## 🧩 SDK Usage

Install SDK:

```bash
cd sdk
python -m pip install -e .
```

Run example:

```bash
python examples/minimal_controller.py
```

---

## 🔁 Typical Workflow

1. Start backend (VRPN → mocap → cf_bridge)
2. Run SDK controller
3. Iterate on control algorithms only (no backend restart needed)

---

## ⚠️ Troubleshooting

### No topics visible

```bash
ros2 node list
ros2 daemon status
```

---

### No `/mocap/...` topics

* Check VRPN topics
* Verify `mocap.yaml`

---

### Drone not responding

* Check Crazyradio
* Verify URI
* Test cflib:

```bash
python3 -c "import cflib"
```

---

### Rebuild backend

```bash
rm -rf build install log
colcon build --symlink-install
```

---

## 🛑 Shutdown

Stop in order:

1. `cf_bridge`
2. `mocap_bridge_ros2`
3. VRPN client

---

## 📌 Notes

* Backend must be running before SDK
* SDK does NOT start ROS nodes
* Designed for real-time control experiments
