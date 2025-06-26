# ROS 2 Fundamentals & Orange Pi Zero 3 → ESP32‑S3 Workshop

> **Goal for this repository**
> *Build a rock‑solid foundation for ROS 2 on ARM SBCs and prepare for micro‑ROS on an ESP32‑S3 DevKit RGB LED demo.*
> This training is structured into **four major phases**:
>
> 1. **Linux & Kernel Fundamentals**: Learn how Linux distributions and kernels relate to ROS 2 (Jazzy Jalisco), and how to install them on a headless server (e.g. Orange Pi Zero 3). Understand kernel modules, compatibility, and simulation capability in minimal installs.
> 2. **Core ROS 2 Concepts**: Set up a Hello World publisher/subscriber, understand topics, QoS, and network-level details (UDP, multicast, IP, ports, etc.).
> 3. **micro-ROS Communication**: Connect and program an ESP32-S3 DevKit using micro-ROS to receive ROS 2 messages and actuate an RGB LED. Explore agent/client setup, transport layers (UDP/Serial), and real-time OS integration.
> 4. **ROS 2 Simulation with AMCL**: Introduce simulation using Gazebo or RViz, load a provided AMCL (Adaptive Monte Carlo Localization) map, and demonstrate basic 2D localization and movement in a known environment.

---

## 0  Repository topology

```
ros2‑esp32‑training/
├── README.md               ← **THIS** doc
├── docs/
│   ├── architecture.md     ← diagrams & deep dives (TBD)
│   └── networking.md       ← DDS/RTPS notes (TBD)
├── ros2_ws/                ← classic ROS 2 Colcon workspace
│   └── src/
│       └── demo_nodes_cpp/ ← talker / listener sources copied for reference
└── scripts/
    ├── install_ros2_jazzy.sh
    └── test_talker_listener.sh
```

---

## 1  Host hardware & OS choices

| SBC                                   | Recommended OS                       | Kernel        | Why we pick it                                                                                                                                                                     |
| ------------------------------------- | ------------------------------------ | ------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Orange Pi Zero 3 (Allwinner H618)** | Armbian 25 *or* Ubuntu 24.04 (Noble) | **6.1.x LTS** | 6.1 is a long‑term‑support kernel (EOL Dec‑2026) already **main‑line** for H618, giving modern I²C/SPI drivers, ext4 improvements, eBPF, and PREEMPT\_RT patches in the same tree. |

> Check your running kernel:
>
> ```bash
> uname -a   # expect ... 6.1.*
> ```

### Armbian vs Ubuntu Noble

|                | Armbian 25.x                                              | Ubuntu 24.04 (server)                           |
| -------------- | --------------------------------------------------------- | ----------------------------------------------- |
| Image size     | minimal (headless)                                        | larger                                          |
| Kernel cadence | bleeding‑edge backports (e.g. 6.12‑rc on nightly)         | distro‑maintained 6.1.x                         |
| Default user   | *root/1234* then *armbian-config*                         | *ubuntu/ubuntu*                                 |
| Pros           | board tweaks (u‑boot overlays, HW accel), great CLI tools | official ROS 2 binaries are built **for Noble** |
| Cons           | ROS 2 binaries require manual repo pins                   | slightly heavier                                |

**Rule of thumb:** pick **Ubuntu 24.04** if you want turnkey ROS 2 packages; choose **Armbian** for bare‑bones IoT images or if you need extra peripherals.

Yes, even Ubuntu Server (headless) can support simulation, as long as you install headless-compatible versions of RViz or Gazebo, or connect via X11/SSH with GUI forwarding or use a VNC server.

---

## 2  Installing ROS 2 Jazzy Jalisco (on Armbian 25.5 Noble)

> **Goal:** end up with a minimal but *ROS‑ready* CLI image that can build & run the demo talker/listener and future micro‑ROS agents.
>
> Tested on `6.12.30-current-sunxi64` (Orange Pi Zero 3).

### 2.1  System prep & locale

```bash
sudo apt update && sudo apt upgrade -y   # pull latest security fixes
sudo apt install -y curl gnupg lsb-release software-properties-common locales

# (optional) set UTF‑8 locale to avoid Python warnings in colcon
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 2.2  Add the ROS 2 repository (keyring‑based)

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  | sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list
```

Check that APT sees the repo:

```bash
apt policy ros-jazzy-desktop | grep Candidate    # should show a version string
```

### 2.3  Install a ROS 2 *profile*

Pick **one** of these meta‑packages:

| Profile                  | Packages                          | Footprint | Use‑case                                     |
| ------------------------ | --------------------------------- | --------- | -------------------------------------------- |
| `ros-jazzy-ros-base`     | core client libs + CLI + launch   | \~280 MB  | headless SBCs, micro‑ROS agent only          |
| `ros-jazzy-desktop`      | base + RViz2 + demos              | \~650 MB  | you’ll VNC/ssh‑X into the board occasionally |
| `ros-jazzy-desktop-full` | desktop + simulators + perception | >1 GB     | if the SBC *is* your dev machine             |

For a headless robot brain we usually install **ros-base**:

```bash
sudo apt update
sudo apt install -y ros-jazzy-ros-base    # or ros-jazzy-desktop
```

### 2.4  Bootstrap the environment

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.5  Developer tools & dependency resolver

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-argcomplete

sudo rosdep init        # one‑time as root
rosdep update           # as any user; grabs package index
```

### 2.6  Smoke‑test the installation (talker ↔ listener)

Open **Terminal A** (SSH session #1):

```bash
ros2 run demo_nodes_cpp talker
```

Open **Terminal B** (SSH session #2):

```bash
ros2 run demo_nodes_cpp listener
```

You should see incrementing `Hello World: N` messages in Terminal B. If not, ensure UDP multicast 239.255.0.1:7400 is allowed (check `ss -ulnp | grep 7400`).

### 2.7  (Option) Headless simulation capability

Even without a desktop you can:

```bash
# Gazebo libraries only (no GUI)
sudo apt install -y ros-jazzy-gazebo-ros-pkgs ros-jazzy-gazebo-dev

# Launch Gazebo off‑screen
gazebo --headless -s libgazebo_ros_factory.so &
```

Then on your *laptop*, run RViz2 and subscribe to simulation topics (`/tf`, `/scan`, etc.). For lightweight map/localisation tests we’ll use AMCL + rviz via remote.

### 2.8  Create your first catkin‑less workspace (colcon)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
ros2 pkg create --build-type ament_cmake my_cpp_node --dependencies rclcpp std_msgs

colcon build --symlink-install
source install/setup.bash
ros2 run my_cpp_node my_cpp_node
```

> The above steps are scripted in `scripts/install_ros2_jazzy.sh`—feel free to run or copy‑paste pieces while learning.

---

## 3  Create and build your first workspace

  Create and build your first workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-installzy/setup.b
source install/setup.bash
```

Copy the reference talker/listener sources for later hacking:

```bash
ros2 pkg create --build-type ament_cmake demo_nodes_cpp_clone --dependencies rclcpp std_msgs
cp /opt/ros/jazzy/share/demo_nodes_cpp/src/talker.cpp src/demo_nodes_cpp_clone/
cp /opt/ros/jazzy/share/demo_nodes_cpp/src/listener.cpp src/demo_nodes_cpp_clone/
colcon build --packages-select demo_nodes_cpp_clone
```

---

## 4  Running the classic *Hello World* (Talker ↔ Listener)

### Terminal 1 – Publisher

```bash
source ~/ros2_ws/install/setup.bash
ros2 run demo_nodes_cpp_clone talker
```

### Terminal 2 – Subscriber

```bash
source ~/ros2_ws/install/setup.bash
ros2 run demo_nodes_cpp_clone listener
```

> You should see incrementing "Hello World: *#*" messages on the listener side.

---

## 5  What actually happens on the wire?

| Concept          | Implementation in ROS 2                                                     |
| ---------------- | --------------------------------------------------------------------------- |
| **Middleware**   | DDS/RTPS (Fast‑DDS by default on Jazzy)                                     |
| **Transport**    | UDP unicast **+** UDP multicast for discovery; TCP/SHM optional             |
| **Discovery**    | Multicast on `239.255.0.1:7400` (port mapping `7400 + 250*domain + offset`) |
| **Logical unit** | *Node* (executable), which owns *Publishers* & *Subscribers*                |
| **Message bus**  | *Topics* (e.g. `/chatter`)                                                  |
| **Isolation**    | `ROS_DOMAIN_ID` env var (0‑232) prevents cross‑talk on shared LAN           |

### Do I need to set IPs or open ports?

* On a **flat Layer‑2 network**: **No** — DDS uses multicast to discover peers.
* Across subnets/VLANs or Wi‑Fi ↔ Ethernet bridges: you may need to

  1. allow UDP 7400‑7999, or
  2. set `RMW_DISCOVERY_OPTIONS=LOCALHOST` + static peer list, or
  3. use ROS 2 bridge relays.

### QoS cheat‑sheet

| Profile     | Reliability   | History      |
| ----------- | ------------- | ------------ |
| Sensor data | *best effort* | keep last 10 |
| Default     | *reliable*    | keep last    |

> Jazzy ships improved *loaned messages* and zero‑copy intra‑process comms.

---

## 6  Software‑architecture perspective

* **Node = micro‑service** — encapsulates a single responsibility.
* **Topic = event bus** — decouples producer/consumer lifecycles.
* **DDS QoS = contract** — latency vs reliability trade‑offs.
* **Workspace = mono‑repo** — all packages share ament & Colcon metadata.
* **Layering**
  SBC (Orange Pi) ↔ *DDS* ↔ ROS Graph ↔ *micro‑ROS* ↔ ESP32 Firmware.

A sequence diagram is planned in \[`docs/architecture.md`].

---

## 7  Where we stand & next milestones

* ✅  Kernel/OS rationale & ROS 2 Jazzy installation
* ✅  Built and executed *talker ↔ listener*
* ✅  Explored network internals
* 🔜  Cross‑compiling micro‑ROS client for ESP32‑S3 & lighting the RGB LED
* 🔜  Loading and simulating with AMCL map in ROS 2 (localization demo)

> Commit early, commit often. Push this skeleton to GitHub and tag it `v0.1.0-ros2-basics` before moving on.

