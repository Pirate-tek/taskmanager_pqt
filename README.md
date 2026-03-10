The **nav2_taskmanager** package provides a powerful ROS2‑based task manager designed to control robot missions through an intuitive graphical interface built with PyQt. Leveraging ROS2’s modular architecture, it orchestrates navigation, simulation, and mission execution components, allowing users to select waypoints, launch missions, and monitor progress in real time. The PyQt UI offers a modern, responsive experience with clear visual feedback, enabling seamless interaction for both developers and operators.

Two Main Components:

mission_launch.py
 – launches Gazebo simulation + Nav2 navigation stack
 
control_node.py
 – single ROS 2 node that:
Optionally starts the launch file via subprocess
Displays an interactive CLI menu
Sends goals directly to Nav2 via NavigateToPose action client
Performs post-arrival maneuvers (rotate + backup)
Generates mission_report.csv on exit

# With simulation (Gazebo + Nav2 + RViz)
ros2 run nav2_taskmanager control_node launch_sim:=true

# Without simulation (connect to existing Nav2)
ros2 run nav2_taskmanager control_node launch_sim:=false