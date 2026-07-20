# BWSI_QuestLog_Testing
testing and old codes for BWSI quest log

to run the attitude node paste these into the terminal

cd ~/ros2_ws
colcon build --symlink-install --packages-select racecar_neo_ros2_driver
source install/setup.bash
ros2 run racecar_neo_ros2_driver attitude_node

In another terminal

source ~/ros2_ws/install/setup.bash
ros2 topic echo /attitude
