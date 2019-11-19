#! /bin/bash


if [ "$1" == "" ]; then
    echo "Usage: capture.sh (name)"
    exit
fi

printf "Capturing..."
rosrun ivr_assignment image1_capture.py
printf "..."
rosrun ivr_assignment image2_capture.py
printf "done!\n"

mv cam1_yz.png host/testing/captures/cam1_yz_$1.png
mv cam2_xz.png host/testing/captures/cam2_xz_$1.png
echo "Saved capture $1."
