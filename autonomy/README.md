## Build and run autonomy package 


###  In Terminal 1 run
- cd ~/ autonomy_ws
- colcon  build --symlink-install --packages-select autonomy
- source install/local_setup.bash
- ros2 launch asl_tb3_sim root.launch.py

This will start Turtlebot simulator. No GUI should appear.

### In Terminal 2 run
- ros2 launch autonomy_repo heading_control.launch.py

This will start up the heading controller that we just created , and open RVIZ. DO NOT SET A GOAL POSE.
Allow thirty seconds for the code in Terminal 2 to fully start up.


### In Terminal 3 run.
- ros2 run autonomy_repo p3_plot.py

This will command a fixed goal position to test the controller,
and produce a plot that you can submit for grading.

You should see your Turtlebot moving in RVIZ.
- This function will take at least ten seconds to produce a plot.
- After you ’ve successfully produced a plot, try setting a new goal post in RVIZ by selecting the "Goal Pose" button on the top toolbar and then
clicking and dragging in the environment to set a goal heading.

Please include the resulting plot p3_output.png in your write-up.

### Lets consider how adjusting kp affects the resulting motion of the Turtlebot
- Provide a description of the behavior you would expect to see as the proportional gain value goes to zero? 
- What about as it goes to infinity? Can we actually make kp arbitrarily large on a physical robot?