# prerequisites
- ubuntu 20.04
- ros-noetic
- ocs2 : https://github.com/leggedrobotics/ocs2.git
- airbot_sdk: https://github.com/DISCOVER-Robotics/sdk

# complile
```
mkdir ~/airbot_ws/src
git clone https://github.com/DISCOVER-Robotics/sdk.git
mv sdk/ros ~/airbot_ws/src
cd ~/airbot_ws/src
git clone https://github.com/leggedrobotics/ocs2.git
# config ocs2 refer to https://leggedrobotics.github.io/ocs2/
cd ..
catkin build ocs2_airbot ros_interface robot_description
# source
source devel/setup.bash
roslaunch ros_interface airbot_arm.launch end_mode:=none
# another terminal
source devel/setup.bash
roslaunch ocs2_airbot ocs2_airbot.launch 
```