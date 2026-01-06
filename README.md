# 6-DOF Cobot Pro - Project 🚀

**Author:** Salim Samet Kotaş  
**University:** Necmettin Erbakan University, Mechatronics Engineering  
**Status:** Completed (v1.0.0)

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![MoveIt2](https://img.shields.io/badge/MoveIt-2.0-orange)
![Gazebo](https://img.shields.io/badge/Simulation-Gazebo-green)
![Python](https://img.shields.io/badge/GUI-PyQt5-yellow)

## 📖 Project Overview
This project is a comprehensive simulation and control infrastructure designed for a custom 6-DOF collaborative robot arm. It bridges the gap between mechanical design, high-fidelity simulation, and real-time control logic.

### Key Features
* **🧠 Advanced Motion Planning:** Integrated with **MoveIt 2** for obstacle avoidance and trajectory planning.
* **🎮 Custom Control Interface:** A user-friendly **PyQt5 GUI** allowing real-time control, singularity analysis, and safety monitoring.
* **🔌 Custom Controllers:** Implemented low-level **C++ controllers** utilizing `ros2_control` hardware interfaces.
* **🌍 Physics Simulation:** High-fidelity **Gazebo Classic** environment with gravity, collision, and friction physics.
* **📐 Kinematics Engine:** Custom Analytical IK and FK solvers implemented as a MoveIt plugin.

## 🛠️ Tech Stack & Dependencies
* **OS:** Ubuntu 22.04 LTS
* **Middleware:** ROS 2 Humble Hawksbill
* **Simulation:** Gazebo Classic 11
* **Visualization:** RViz 2
* **Languages:** C++ (Controllers/Plugins), Python (GUI/Launch)

## 🚀 Installation & Build

```bash
# 1. Clone the repository
mkdir -p ~/cobot_ws/src
cd ~/cobot_ws/src
git clone [https://github.com/salimsametkotas/6dof-cobot-project.git](https://github.com/salimsametkotas/6dof-cobot-project.git)

# 2. Install dependencies
cd ~/cobot_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Build the workspace
colcon build
source install/setup.bash

🎮 How to Run

The system is modular. Run these commands in separate terminals:

1. Physics Simulation (Gazebo)
ros2 launch cobot_gazebo gazebo.launch.py

2. Motion Planning (MoveIt)
ros2 launch cobot_moveit_config move_group.launch.py use_sim_time:=true

3. Visualization (RViz)
ros2 launch cobot_moveit_config moveit_rviz.launch.py use_sim_time:=true

4. User Interface (GUI)
ros2 run cobot_gui cobot_gui_node
