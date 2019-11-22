# IVR - Patrick Kage / Victor Stoian

Entry points:
  `catkins_ws/src/ivr_assignment/src/q2_1.py`
  `catkins_ws/src/ivr_assignment/src/plot_positions.py`

Docker image with ROS and OpenCV, accessible via a web browser. Not very tested, runs as root: probably insecure.



## Building

```bash
make build
```


## Running

This maps the current directory to /root/host in the docker container, and `catkins_ws/` into the `/root/catkins_ws` folder.

```bash
make launch
```

Then go to http://localhost:6080/ to load up the desktop running in the docker container.


## Executing shell commands in the docker container from the host shell

```bash
make connect
root@docker# Any commands
```


source devel/setup.bash
roslaunch ivr_lab spawn.launch
rostopic pub -1 /robot/joint1_position_controller/command std_msgs/Float64 "data: 1.0"

rosrun ivr_lab image_processing.py
