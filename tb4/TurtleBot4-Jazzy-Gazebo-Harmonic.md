## TurtleBot4 + Nav2 + SLAM pipeline for Jazzy + Gazebo Harmonic on Ubuntu 24.04

### 0: Tutorial Goals
- Launch Gazebo world + TurtleBot4
- Launch SLAM toolbox to create a map
- Launch Nav2 for autonomous navigation
- Optionally, use teleop to drive manually

### 1: Install required packages


```bash
# Add the ROS 2 repository
sudo apt update
sudo apt install -y curl gnupg lsb-release

# Add the ROS key:
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the ROS repository:
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y \
ros-jazzy-turtlebot4-simulator \
ros-jazzy-nav2-bringup \
ros-jazzy-ros-gz \
ros-jazzy-ros-gz-sim \
ros-jazzy-slam-toolbox
```

- turtlebot4-simulator → robot model + Gazebo worlds
- nav2-bringup → Navigation2 stack
- ros-gz & ros-gz-sim → ROS 2 ↔ Gazebo bridge
- slam-toolbox → SLAM for mapping


### 2: Source ROS 2 Jazzy

```bash
source /opt/ros/jazzy/setup.bash
```

Add command to `~/.bashrc` to auto-source:

```bash
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    source ~/.bashrc
```

### 3: Launch Gazebo with TurtleBot4

```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py
```

- Opens Gazebo Harmonic
- Spawns TurtleBot4 in default world
- Starts ROS bridge automatically

### 4: Start SLAM

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch turtlebot4_gz_bringup turtlebot4_slam.launch.py
```

- This launches SLAM Toolbox (online mapping)
- The robot’s lidar data is bridged from Gazebo to ROS 2
- Can start driving the robot around the world to build a map

### 5: Start Navigation (Nav2)

Once we have a map (or while we are mapping):

```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_navigation.launch.py
```

- Launches Nav2 stack
- Uses the existing map or SLAM data
- Supports autonomous navigation with goals

### 6: Teleoperation (drive robot manually)

Install teleop if we don’t have it:

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
```

Run teleop:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- Now we can manually drive TurtleBot4 in Gazebo
- Nav2 can run alongside and navigate autonomously

Note:
- Gazebo Harmonic now uses ros_gz (formerly ignition_ros)
- all ROS topics from sensors come in through /tf, /scan, /odom, etc

Check:
```bash
ros2 topic list
```

### 7: Launch file starting Gazebo, TB4, SLAM, and Nav2 in one go

#### a: Create a workspace for custom launch

```bash
mkdir -p ~/turtlebot4_ws/src
cd ~/turtlebot4_ws/src
```

Clone any necessary TurtleBot4 packages if not already installed via apt

```bash
# Only if you want the source version, otherwise apt packages work
git clone https://github.com/ROBOTIS-GIT/turtlebot4.git
```

Go back to workspace root:

```bash
cd ~/turtlebot4_ws
```

#### b: Create a custom launch package

```bash
ros2 pkg create --build-type ament_python tb4_sim_launch
cd tb4_sim_launch
mkdir launch
```

#### c: Add combined launch file

Create a file: `launch/tb4_full_sim.launch.py`

```python

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to existing TurtleBot4 launch files
    tb4_bringup_dir = get_package_share_directory('turtlebot4_gz_bringup')
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb4_bringup_dir, 'launch', 'turtlebot4_gz.launch.py'))
    )
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb4_bringup_dir, 'launch', 'turtlebot4_slam.launch.py'))
    )
    
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb4_bringup_dir, 'launch', 'turtlebot4_navigation.launch.py'))
    )
    
    return LaunchDescription([
        gazebo_launch,
        slam_launch,
        nav_launch
    ])

```

- gazebo_launch → spawns TurtleBot4 in Gazebo
- slam_launch → SLAM Toolbox (online mapping)
- nav_launch → Nav2 stack

#### d: Build the workspace

```bash
cd ~/turtlebot4_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

#### e: Launch full simulation

```
ros2 launch tb4_sim_launch tb4_full_sim.launch.py
```

The `ros2 launch` command will 

- Open Gazebo Harmonic with TurtleBot4
- Start SLAM Toolbox to build a map
- Launch Nav2 for autonomous navigation

We can now:

- Use `ros2 run teleop_twist_keyboard teleop_twist_keyboard` to drive manually
- Observe mapping in real-time
- Send Nav2 goals via RViz2

### 8: Optional improvements:

- Add remapping to avoid topic conflicts
- Start RViz2 automatically in the launch file
- Add world file argument to switch between Gazebo worlds

We can extend launch file, `tb4_full_sim.launch.py`,  so it:

- Starts RViz2 automatically
- Lets us choose the Gazebo world at launch with a command-line argument

```python
# tb4_full_sim.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch argument to choose Gazebo world
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.world',  # default Gazebo world
        description='Gazebo world to load'
    )
    
    tb4_bringup_dir = get_package_share_directory('turtlebot4_gz_bringup')
    
    # Include Gazebo launch with world argument
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_bringup_dir, 'launch', 'turtlebot4_gz.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )
    
    # SLAM launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_bringup_dir, 'launch', 'turtlebot4_slam.launch.py')
        )
    )
    
    # Nav2 launch
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_bringup_dir, 'launch', 'turtlebot4_navigation.launch.py')
        )
    )
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(tb4_bringup_dir, 'rviz', 'turtlebot4_nav.rviz')]
    )
    
    return LaunchDescription([
        world_arg,
        gazebo_launch,
        slam_launch,
        nav_launch,
        rviz_node
    ])
```

##### Example usage 

```bash
# Launch default empty world
ros2 launch tb4_sim_launch tb4_full_sim.launch.py

# Launch a custom world
ros2 launch tb4_sim_launch tb4_full_sim.launch.py world:=my_custom_world.sdf
```

- RViz2 starts automatically with the TurtleBot4 Nav2 configuration.
- We can switch worlds without editing the file; just provide the world argument.

Now your pipeline is fully integrated: Gazebo, TurtleBot4, SLAM, Nav2, and RViz2 with flexible worlds.

References: https://docs.google.com/document/d/15-2KHiOp0XkMWzcS3h2e2lNmgJg9WxRyfuEwXu0niWg/edit?usp=sharing



