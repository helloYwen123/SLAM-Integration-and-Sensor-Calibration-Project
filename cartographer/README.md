# Cartographer Package
`Cartographer` can generate a 2D Grid Occupancy map using two 2D-Single-Line Lasers or other multiple sensors. Additionally, `Cartographer` has **pure localization** mode which can be used for localization with given 2D map.

## Docker Environment Set-Up
Use `Docker` to build and run `Cartographer` on the outdated hardware system. Due to the limitations of the old hardware, Docker provides a more compatible and modern environment, enabling the deployment of updated applications without being constrained by the legacy system
First, build the Docker image from the same directory as `Dockerfile`.
```bash
docker build -t <your_image_name> .

```
To create and run the Docker container while mapping a local directory to a directory inside the container, use:
```bash
docker run -it --name <project_container> -v /path/to/local/dir:/path/in/container <your_image_name>
```
Here paths are supposed to be the path of your local workspace and one in Docker Container.

Setting Up ROS Communication Between Docker Container and Host. Basically, You need to modify the `.bashrc` file and add following lines to keep `ROS_MASTER_URI` and `ROS_IP` consistent between Host and Container.
```bash
export ROS_MASTER_URI=http://<host-ip>:11311
export ROS_IP=<container-ip>
```
Finally, start and attach container in your terminal.
```bash
docker start <container-name>
docker attach <container-name>
```
For the lab-tested robot, I set the container name to `cartotrucker` and the image name to `darko_carto`

## Building & Installation

First, Use these commands to install the tools in your `Docker Env`:
```bash
sudo apt-get update
sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
```
Then use `rosdep` to install the required packages.
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```
Cartographer uses the abseil-cpp library that needs to be manually installed using this script:
```bash
src/cartographer/scripts/install_abseil.sh
```
And then build and install:
```bash
catkin_make_isolated --install --use-ninja
```

## Running

After build and Installation need you to run `cartographer-slam` on our real-time harware.

Firstly, you can install `terminator`, which is a popular terminal emulator for Linux that allows users to efficiently manage multiple terminal windows within a single interface.
```bash
sudo apt install terminator
``` 
Copy the `.lua` in `configuration_file` to`<path-to-workspace>/install_isolated/share/cartographer_ros/configuration_file` 

Copy `.launch` file in `launch_file` to `<path-to-workspace>/install_isolated/share/cartographer_ros/launch`

And then source the environment and run launch file in Docker.
```bash
source /opt/ros/melodic/setup.bash
source /root/catkin_ws/devel_isolated/setup.bash
roslaunch cartographer_ros kirobot.launch
```
Because we're using `terminator` tool, we can just create a new windows in same interface and run:
```bash
rviz
```

## Maps Save
Here we can also save generated map from SLAM. Run the script file `map.sh`  in this Package .
```bash
./map.sh
```
After this, you will obtain a .pbstream map file in `Cartographer` format. Additionally, you can run the following commands to convert it to the standard ROS map format. This allows you to perform more general motion planning using a ROS map without relying on `Cartographer`.
```bash
rosrun cartographer_ros cartographer_pbstream_to_ros_map \
-pbstream_filename=<path-to-pbstream> \
-map_filestem=<path-to-map>
```

## Pure Localization Model
`cartographer` offers possibility to do Pure Localization, which make robot focus on Localization and disable Mapping.
To do so, you need to run another Launch file in Package.
```bash
roslaunch cartographer_ros kirobot_localization.launch
```
The Pure Localization model enables the robot to localize itself within a known map. Experimental results demonstrate that it outperforms the SLAM mode in accuracy.


