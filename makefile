PWD = $(shell pwd)

all: launch

launch:
	docker run --rm -p 6080:80 -v /dev/shm:/dev/shm -v "$(PWD)":/root/host -v "$(PWD)/catkins_ws":/root/catkins_ws --name ubuntu-ros-opencv ubuntu-ros-opencv


connect:
	docker exec -it ubuntu-ros-opencv bash

build:
	docker build . -t ubuntu-ros-opencv
