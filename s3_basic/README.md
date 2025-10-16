## Run s3_basic package with rviz 

### build s3_basic package and launch gazebo in terminal 1
- cd ~/autonomy_ws
- colcon build --symlink-install
- source install/setup.bash
- ros2 launch asl_tb3_sim signs.launch.py

### launch control_node and rviz in terminal 2
- source install/setup.bash
- ros2 launch s3_basic section3.launch.py

### stop the robot when appropriate in terminal 3
- source install/setup.bash
- ros2 topic pub /kill std_msgs/msg/Bool data:\ true -1
