Catch robot
============
This is a ROS package that is used to validate my planning algorithm implementation. It consists of 3 nodes: running\_robot catch\_robot and pub\_map, which kinds of explain their function by their names.
Once complied, run:
```
rosrun catch_point_robot pub_map test/map1.txt
rosrun catch_point_robot running_robot
rosrun catch_point_robot catch_robot
```
A launch file might be useful. I might add one some time later.
There is indeed a launch file now which helps to visualize the process in rviz.
```
roslaunch catch_point_robot start_rviz.launch
```
