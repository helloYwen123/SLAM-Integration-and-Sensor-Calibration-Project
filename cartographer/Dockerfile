FROM ubuntu:18.04
FROM ros:melodic-ros-base

# Install necessary ROS packages
RUN apt-get update && apt-get install -y \
    	ros-melodic-catkin\
	cmake\
	git\
	g++\
	libboost-all-dev\
	libcairo2-dev\
	ninja-build\
	protobuf-compiler\
	python3-sphinx \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/catkin_ws





