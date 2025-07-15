**Install Gazebo & ROS 2 Gazebo Integration**
sudo apt update
sudo apt install gazebo ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

**Install TurtleBot3 Simulations (includes world + URDF + robot configs)**
sudo apt install ros-humble-turtlebot3-gazebo

**Set TurtleBot3 Model**
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc

**Downloading the Custom World**
cd ~/ros2_ws/src
git clone https://github.com/leonhartyao/gazebo_models_worlds_collection.git

**Set your robot model and Gazebo model paths**
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/ros2_ws/src/gazebo_models_worlds_collection/models


**Clone my repo into ros2_ws/src-**
cd ~/ros2_ws/src
git clone YOUR_REPO_URL    

**Build the workspace**
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
      
**Launch the simulation**
ros2 launch my_robot_sim my_world_launch.py

**Run keyboard teleop**
**Install teleop package once (if not present)**
sudo apt install ros-humble-teleop-twist-keyboard
source ~/ros2_ws/install/setup.bash

**Run teleop node**
ros2 run teleop_twist_keyboard teleop_twist_keyboard
