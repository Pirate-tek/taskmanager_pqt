# Nav2 Taskmanager

The **nav2_taskmanager** package provides a powerful ROS2‑based task manager designed to control robot missions through an intuitive graphical interface built with PyQt. Leveraging ROS2’s modular architecture, it orchestrates navigation, simulation, and mission execution components, allowing users to select waypoints, launch missions, and monitor progress in real time. The PyQt UI offers a modern, responsive experience with clear visual feedback, enabling seamless interaction for both developers and operators.

## Main Components:

- **mission_launch.py**: Launches Gazebo simulation with the Dynominion robot and Nav2 stack.
- **gui_node.py**: PyQt interface for mission control, sequencing, and manual joystick operation.
- **mission_executor_node.py**: Action server that translates high-level tasks into Nav2 goals.
- **nearest_waypoint_node.py**: Service provider that calculates the closest waypoint for the robot.

## ROS2 Interface

### Topics
| Topic | Description |
| :--- | :--- |
| `/cmd_vel` | Publishes velocity commands to the robot base for manual control. |
| `/amcl_pose` | Provides the robot current estimated position on the map accurately. |

### Services
| Service | Description |
| :--- | :--- |
| `/get_nearest_waypoint` | Triggers calculation to find the closest predefined waypoint to robot. |
| `/stop_mission` | Service to immediately stop all robot movement and mission tasks. |

### Actions
| Action | Description |
| :--- | :--- |
| `mission_task` | Action server for executing complex waypoint navigation and rotation maneuvers. |
| `navigate_to_pose` | Standard Nav2 action used by the executor to reach goals. |

## Usage

### Launching the GUI
```bash
ros2 run taskmanager_pqt gui_node
```

### With simulation (Gazebo + Nav2 + RViz)
1. Start Simulation button in GUI (or manual launch):
```bash
ros2 launch taskmanager_pqt mission_launch.py
```
2. Start Mission Executor button in GUI (starts executor and nearest_wp nodes).