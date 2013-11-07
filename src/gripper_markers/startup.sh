#!/bin/sh

gnome-terminal --tab -e "bash -c 'roscore'" \
--tab -e "bash -c 'sleep 5 && roslaunch pr2_gazebo pr2_table_object.launch'" \
--tab -e "bash -c 'sleep 25 && roslaunch pr2_interactive_manipulation pr2_interactive_manipulation_robot.launch sim:=true'" \
--tab -e "bash -c 'sleep 45 && roslaunch gripper_markers ik_servers.launch'" \
--tab -e "bash -c 'sleep 50 && rosrun gripper_markers gripper_markers.py'" \
--tab -e "bash -c 'sleep 55 && rqt'"
