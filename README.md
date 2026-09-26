# Mouse_controller
# 🏎️ ROS 2 Custom Pure Pursuit: Mouse-Driven Navigation for Bicycle Kinematics

This project implements a **custom Pure Pursuit navigation stack** for a non-holonomic robot (bicycle steering kinematics) in ROS 2. Instead of relying on pre-computed paths, the robot dynamically follows trajectories generated in real-time simply by clicking on the OpenCV interface.

## 🎯 Project Goals & Features
* **Interactive Navigation:** Translate mouse clicks from OpenCV interface into executable trajectories visible on RViz.
* **Advanced Pure Pursuit:** A custom C++ controller that handles lookahead distances, turning radius physical limits, and **geometric reverse logic** (the robot intelligently drives backward when the target is behind it).
* **Dockerized & Ready:** The entire workspace is containerized for instant reproducibility.

## 🤝 Acknowledgments & Origins
This project builds upon the foundational models provided by the official [ros2_control_demos](https://control.ros.org/jazzy/doc/ros2_control_demos/doc/index.html) repository. Specifically, the robot's URDF, 3D description (`ros2_control_demo_description`), and the base structure for `example_11` were extracted and heavily modified to integrate my custom trajectory and pursuit nodes into a clean, isolated workspace.

## 🚀 How to Run (Instructions)

### 1. Clone the Repository
Get the code onto your local machine by running:

git clone https://github.com/dvernice01/Mouse_controller.git

cd Mouse_controller

### 2. Build the Docker Image

docker build -t mouse_controller .

./run.sh

### 3. Build the ROS 2 Workspace

colcon build

source install/setup.bash

### 4. Run the Controller :)

ros2 launch ros2_control_demo_example_11 carlikebot.launch.py

## Once the simulation is running, use the white "Mouse controller" window to command the robot. Here are the controls:

🖱️ Draw a Trajectory: Hold down the Left Mouse Button and drag across the window to draw a custom path for the robot to follow.

⏫ Drive Straight: Double-click the Left Mouse Button to send the robot continuously forward.

🛑 Emergency Stop: Click the Middle Mouse Button (scroll wheel) to halt the robot immediately.

🔄 Reverse Trajectory: Hold Ctrl + Left Mouse Button and drag to draw a path for the robot to follow in reverse gear.

⏬ Reverse Straight: Hold Ctrl + Double-click the Left Mouse Button to send the robot continuously backward.

🧹 Clear Trajectories: Press the Delete key (Canc) to erase all previously drawn paths (Note: you might need to press it a couple of times to register).

📍 Reset Position: Press the Spacebar to respawn the robot at its starting position (Note: you might need to press it a couple of times to register).
