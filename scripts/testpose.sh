#! /bin/bash

trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

./neutral.sh

sleep 0.5

echo "starting captures..."

rosrun ivr_assignment q2_1.py &

read -p "Press enter to continue"

echo "setting."
rostopic pub -1 /robot/joint1_position_controller/command std_msgs/Float64 "data: 0"  &
rostopic pub -1 /robot/joint2_position_controller/command std_msgs/Float64 "data: 0" &
rostopic pub -1 /robot/joint3_position_controller/command std_msgs/Float64 "data: 0" &
rostopic pub -1 /robot/joint4_position_controller/command std_msgs/Float64 "data: 1.5707"

echo "captures are now valid"

read -p "Press enter to exit"

