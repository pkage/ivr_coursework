#! /bin/bash

rostopic pub -1 /robot/joint2_position_controller/command std_msgs/Float64 "data: -1.5707"

