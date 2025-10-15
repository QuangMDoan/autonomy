# Reflection 

### Create File: s2_basic/scripts/constant_control.py

Make constant_control.py has execute permission

<code>
    chmod +x constant_control.py 
</code>

### Write a ROS Node​, see constant_control.py 

### Register Your Node with CMake​

Add the 2 lines to s2_basic/CMakeLists.txt.

<code>

install(PROGRAMS
    scripts/constant_control.py
    DESTINATION lib/${PROJECT_NAME}
)

</code>

### Build the package. Make sure adding the following lines

- which add runtime dependencies for rclpy and std_msgs.

<code>
    colcon build --symlink-install
</code>


### Run the ROS Node​

<code>

source ~/autonomy_ws/install/setup.bash 
ros2 run s2_basic constant_control.py

</code>

### In an separate terminal run the simulator

<code>
ros2 launch asl_tb3_sim signs.launch.py
</code>

### In an separate terminal list the topics and stop the bot

- list all topics
<code>
ros2 topic list
</code>


- Stop the bot

<code>
ros2 topic pub /kill std_msgs/msg/Bool data:\ true -1
</code>
