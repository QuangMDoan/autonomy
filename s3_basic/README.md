## Run s3_basic package with rviz 

### terminal 1
- ros2 launch asl_tb3_sim signs.launch.py

### terminal 2
- ros2 launch s3_basic section3.launch.py

Inspect both sim and rviz windows

### terminal 3
- ros2 topic pub /kill std_msgs/msg/Bool data:\ true -1
