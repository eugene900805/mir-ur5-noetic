﻿# mir-ur5-noetic with moveit
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
