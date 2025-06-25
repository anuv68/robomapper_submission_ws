# robomapper_description

This package contains the robot description, launch files, and controller configurations for a 4-wheeled differential drive robot simulated using ROS 2 Humble, Gazebo, and RViz2.

## Package Structure

robomapper_description/
├── urdf/ # URDF model of the robot
│ └── robomapper_bot.urdf
├── config/ # Controller configuration files
│ ├── controller_manager.yaml
│ └── diff_drive_controller.yaml
├── launch/ # Gazebo launch file
│ └── gazebo.launch.py
├── test/ # Linting tests (optional)
├── setup.py, package.xml # Package metadata

## Build Instructions

Clone this package inside a ROS 2 workspace and build:
cd ~/robomapper_submission_ws
colcon build --symlink-install
source install/setup.bash

Running the Simulation
To launch the robot in Gazebo:
ros2 launch robomapper_description gazebo.launch.py

Teleoperation
Open a new terminal and run:
source ~/robomapper_submission_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

Dependencies
Make sure required ROS 2 packages are installed:

sudo apt update
rosdep install --from-paths src --ignore-src -r -y

---

You can now:

nano README.md
Paste that in, save, then:

git add README.md
git commit -m "Add minimal README for robomapper_description"
git push
