#! /bin/sh

echo "setting to [0,1,-1,1]"
rostopic pub -1 /robot/joint2_position_controller/command std_msgs/Float64 "data: 1.0"  &
rostopic pub -1 /robot/joint3_position_controller/command std_msgs/Float64 "data: -1.0" &
rostopic pub -1 /robot/joint4_position_controller/command std_msgs/Float64 "data: 1.0"

