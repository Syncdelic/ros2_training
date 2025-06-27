# ROS 2 Jazzy Jalisco — Quick‑Start README

A concise path from a clean OS install to a running Navigation2 simulation, with the **core ROS 2 concepts and CLI workflow you’ll use every day**.

---

\## 1 Why Armbian 25 (arm64) *or* Ubuntu 24.04 (x86₆₄)

|                     | Ubuntu 24.04 LTS                          | Armbian 25 (Noble arm64)                               |
| ------------------- | ----------------------------------------- | ------------------------------------------------------ |
| **Kernel**          | 6.8 generic; optional PREEMPT‑RT via repo | 6.8‑rt **enabled by default** (low‑jitter motor loops) |
| **Package base**    | Official Canonical packages               | Same Noble packages + SBC tweaks                       |
| **Target hardware** | Laptops / NUCs                            | Raspberry Pi, Jetson, RK3588, etc.                     |

Both distros ship the libraries ROS 2 Jazzy depends on and allow deterministic scheduling (RT patches) for real‑time control on a rover. Choose Ubuntu for x86 dev boxes, Armbian for ARM SBCs on the robot.

---

\## 2 Install ROS 2 Jazzy Jalisco
Follow the official guide (works for both Ubuntu 24.04 and Armbian 25):
[https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

After finishing, add to `~/.bashrc`:

```bash
source /opt/ros/jazzy/setup.bash
```

---

\## 3 ROS 2 Fundamentals

### 3.1 Concepts

| Term          | What it is                               | Example                                        |
| ------------- | ---------------------------------------- | ---------------------------------------------- |
| **Node**      | Independent process that does one job    | `/locomotion_node` publishes wheel speeds      |
| **Topic**     | Stream of messages (pub‑sub)             | `/cmd_vel` geometry\_msgs/Twist                |
| **Service**   | Synchronous request/response             | `/reset_odometry` std\_srvs/Empty              |
| **Action**    | Long‑running goal with feedback & cancel | `/navigate_to_pose` nav2\_msgs/ NavigateToPose |
| **Parameter** | Runtime config value scoped to a node    | `planner.max_accel:=2.0`                       |
| **Launch**    | Python file that starts many nodes       | `ros2 launch my_robot bringup.launch.py`       |

### 3.2 Why ROS 2 > ROS 1 (Noetic)

* **No ROS master** – discovery via DDS → nodes can start/stop anytime.
* **QoS profiles** – choose reliability per topic (reliable vs best‑effort, transient local, etc.).
* **Real‑time friendly** – lock‑free middle‑ware + RT kernels.
* **Security** – SROS 2, DDS‑Secure.
* **ROS 2 CLI** – richer, modular sub‑commands (`ros2 <verb>`).

---

\## 4 CLI Workflow Cheat‑Sheet (Hello‑World demo)

> We’ll create a tiny publisher (**talker**) and subscriber (**listener**) then explore the CLI—including the fun `ros2 wtf` doctor.

1. **Create workspace & package**

```bash
mkdir -p ~/mdrs_ws/src && cd ~/mdrs_ws
ros2 pkg create --build-type ament_python py_demo
```

2. **Add talker.py** (`py_demo/py_demo/talker.py`)

```python
import rclpy, time
from rclpy.node import Node
from std_msgs.msg import String
class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.create_timer(1.0, self.timer_cb)
    def timer_cb(self):
        msg = String(data=f"hello at {time.time():.0f}")
        self.pub.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

def main():
    rclpy.init()
    rclpy.spin(Talker())
    rclpy.shutdown()
if __name__ == '__main__':
    main()
```

3. **Add listener.py** (`py_demo/py_demo/listener.py`)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.create_subscription(String, 'chatter', self.cb, 10)
    def cb(self, msg):
        self.get_logger().info(f"I heard: {msg.data}")

def main():
    rclpy.init()
    rclpy.spin(Listener())
    rclpy.shutdown()
if __name__ == '__main__':
    main()
```

4. **Declare executables** in `setup.py`:

```python
    entry_points={
        'console_scripts': [
            'talker = py_demo.talker:main',
            'listener = py_demo.listener:main',
        ],
    },
```

5. **Build & source**

```bash
cd ~/mdrs_ws && colcon build --symlink-install
source install/setup.bash
```

6. **Run in two terminals**

```bash
ros2 run py_demo talker   # T1
ros2 run py_demo listener # T2
```

7. **Explore with CLI**

```bash
ros2 node list            # /talker /listener
ros2 topic list           # /chatter
ros2 topic echo /chatter  # See messages
ros2 wtf                  # Doctor checks env & graph
```

---

\## 5 Launch Both Nodes (single command)
Create `py_demo/launch/talker_listener.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='py_demo', executable='talker', name='talker'),
        Node(package='py_demo', executable='listener', name='listener'),
    ])
```

Run:

```bash
ros2 launch py_demo talker_listener.launch.py
```

Both nodes now start together; verify with `ros2 node list`.

---

\## 6 Navigation2 Simulation Quick‑Test

```bash
# Install
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
# For TurtleBot3 on Gazebo Classic (Iron‑ or older → switch pkg names)
export TURTLEBOT3_MODEL=waffle
# Launch sim + Nav2 + RViz & Gazebo GUI
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

In RViz, click **2D Nav Goal** → robot will plan & move. Use CLI to watch:

```bash
ros2 action list                    # Should list /navigate_to_pose
ros2 action info /navigate_to_pose  # See feedback & result types
```

> Next steps: swap TurtleBot for your rover URDF, wire up sensors, tune Nav2 params — or dive into micro‑ROS for MCU integration.

