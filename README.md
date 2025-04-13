# autonav_Crazyflie_Turtlebot3

**Multi-Agent Robot Sensing Simulation and also real-world deployment**

This project simulates autonomous navigation and pollutant density mapping using multiple robots, in this case Turtlebot3 as UGV and Crazyflie 2.x as UAV.
It leverages ROS (Robot Operating System) and Gazebo to model collaborative sensing behaviors in a shared environment. This github repo should help you integrate Crazyflie and Turtlebot3 in the capsulated ROS environtment.

---

## 🧠 Project Overview

The system is designed to:

- Deploy multiple autonomous agents for environmental sensing.
- Map pollutant density using sensor data.
- Visualize robot interactions and data flow through ROS graphs.

---

## 📁 Repository Structure

- `src/` – Core ROS nodes and logic.
- `scripts/` – Utility scripts for robot control and data processing.
- `launch/` – ROS launch files to initialize simulations.
- `param/` – Configuration files for robot parameters.
- `worlds/` – Gazebo simulation environments.
- `cache/` – Cached data and logs.
- `rosgraph.png`, `rosgraph_debug.png`, `rosgraph_debugokey.png` – Visual representations of the ROS node graph.
- `PENGUKURAN DAN PEMETAAN DENSITAS POLUTAN.pptx` – Presentation detailing pollutant density measurement and mapping.
- `CMakeLists.txt` & `package.xml` – ROS build and package configuration files.

---

## 🚀 Getting Started

1. **Prerequisites**:
   - ROS (Robot Operating System) installed.
   - Gazebo simulator set up.

2. **Installation**:
   - Clone the repository:
     ```bash
     git clone https://github.com/Adrian-Jotham/autonav_CF_TB.git
     ```
   - Navigate to the project directory and build:
     ```bash
     cd autonav_CF_TB
     catkin_make
     ```

3. **Running the Simulation**:
   - Launch the simulation environment:
     ```bash
     roslaunch launch/your_launch_file.launch
     ```
   - Replace `your_launch_file.launch` with the appropriate launch file name.
   - Real World Deployment 
      Please look for "tb_cf.launch" launcher file
   - Random Simulation and Implementation also provided in the launch file utilizing different random movements.

---

## 📊 Visualization

The included ROS graph images (`rosgraph.png`, `rosgraph_debug.png`, `rosgraph_debugokey.png`) provide insights into the node and topic architecture of the system.

---

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

## 🤝 Acknowledgments

Developed by Adrian Jotham.

For more information, refer to the presentation: `PENGUKURAN DAN PEMETAAN DENSITAS POLUTAN.pptx`.
