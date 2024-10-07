# Eurobot-2025-Navigation

## To open and remake project
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch navigation_run sim.launch

##  To just open project
source devel/setup.bash
roslaunch navigation_run sim.launch

## To open rival
source devel/setup.bash
rosrun fake_rival fake_rival_main