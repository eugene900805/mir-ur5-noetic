# mir-ur5-noetic with moveit

### Preliminaries

If you haven't already installed ROS on your PC, you need to add the ROS apt
repository. This step is necessary for either binary or source install.

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
```

### Binary install

For a binary install, it suffices to run this command:

```bash
sudo apt install ros-noetic-mir-robot
sudo apt install ros-noetic-moveit
sudo apt-get install ros-noetic-object-recognition-msgs
```

See the tables at the end of this README for a list of ROS distros for which
binary packages are available.

### Source install

For a source install, run the commands below instead of the command from the
"binary install" section.

```bash
# create a catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/

# clone mir_robot into the catkin workspace
git clone https://github.com/eugene900805/mir_robot.git
git clone https://github.com/eugene900805/universal_robot.git
git clone https://github.com/eugene900805/moveit_config.git
git clone https://github.com/eugene900805/Universal_Robots_ROS_Driver.git
git clone https://github.com/eugene900805/robotiq_description.git
git clone https://github.com/eugene900805/gazebo-pkgs.git
git clone https://github.com/eugene900805/general-message-pkgs.git

# use rosdep to install all dependencies (including ROS itself)
sudo apt-get update -qq
sudo apt-get install -qq -y python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths ./ -i -y --rosdistro noetic

# build all packages in the catkin workspace
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
catkin_init_workspace
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebugInfo
```

### Simulation

Single Robot
```bash
### gazebo:
roslaunch mir_gazebo mir_maze_world.launch
rosservice call /gazebo/unpause_physics   # or click the "start" button in the Gazebo GUI

### localization:
roslaunch mir_navigation amcl.launch initial_pose_x:=10.0 initial_pose_y:=10.0
# or alternatively: roslaunch mir_gazebo fake_localization.launch delta_x:=-10.0 delta_y:=-10.0

# navigation:
roslaunch mir_navigation start_planner.launch \
    map_file:=$(rospack find mir_gazebo)/maps/maze.yaml \
    virtual_walls_map_file:=$(rospack find mir_gazebo)/maps/maze_virtual_walls.yaml
rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz

# moveit
roslaunch mir_ur5_gripper_moveit_config move_group.launch limited:=true
roslaunch mir_ur5_gripper_moveit_config moveit_rviz.launch config:=true
```

Multiple Robots
```bash
# start Gazebo + first MiR
roslaunch mir_gazebo mir_maze_world.launch tf_prefix:=mir

# first MiR: start localization, navigation + rviz
roslaunch mir_navigation amcl.launch initial_pose_x:=10.0 initial_pose_y:=10.0 tf_prefix:=mir
roslaunch mir_navigation start_planner.launch \
        map_file:=$(rospack find mir_gazebo)/maps/maze.yaml \
        virtual_walls_map_file:=$(rospack find mir_gazebo)/maps/maze_virtual_walls.yaml prefix:=mir/
ROS_NAMESPACE=mir rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz

# spawn second MiR into Gazebo
roslaunch mir_gazebo mir_gazebo_common.launch robot_x:=-2 robot_y:=-2 tf_prefix:=mir2 model_name:=mir2 __ns:=mir2

# second MiR: start localization, navigation + rviz
roslaunch mir_navigation amcl.launch initial_pose_x:=8.0 initial_pose_y:=8.0 tf_prefix:=mir2
roslaunch mir_navigation start_planner.launch \
        map_file:=$(rospack find mir_gazebo)/maps/maze.yaml \
        virtual_walls_map_file:=$(rospack find mir_gazebo)/maps/maze_virtual_walls.yaml prefix:=mir2/
ROS_NAMESPACE=mir2 rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz

# moveit first MiR
roslaunch mir_mir_ur5_gripper_moveit_config move_group.launch
roslaunch mir_mir_ur5_gripper_moveit_config moveit_rviz.launch config:=true

# moveit first MiR
roslaunch mir2_mir_ur5_gripper_moveit_config move_group.launch
roslaunch mir2_mir_ur5_gripper_moveit_config moveit_rviz.launch config:=true
```

### Real
```bash
# load ur5
roslaunch ur_robot_driver ur5_bringup.launch limited:=true robot_ip:=192.168.0.102
roslaunch real_ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true

# load MiR
roslaunch mir_driver mir.launch
roslaunch mir_navigation amcl.launch initial_pose_x:=10.0 initial_pose_y:=10.0
roslaunch mir_navigation start_planner.launch map_file:=$(rospack find mir_gazebo)/maps/lab.yaml 
rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz 
```
