# map folder
map_dir="${HOME}/catkin_ws/map"
map_name=$(date +%Y%m%d_%H-%M-%S) 
# check if the folder exist
if [ ! -d "$map_dir" ];then
  echo "no folder!! , create new one!!"
  mkdir -p $map_dir
fi

# stop slam
rosservice call /finish_trajectory 0
# generate map
rosservice call /write_state "{filename: '$map_dir/$map_name.pbstream'}"

#rosrun cartographer_ros cartographer_pbstream_to_ros_map \
#-pbstream_filename=$map_dir/$map_name.pbstream \
#-map_filestem=$map_dir/$map_name

# rosrun map_server map_server /home/{yourname}/catkin_ws/map/{yourmapname}.yaml
# <node name="map_server" pkg="map_server" type="map_server" args="/home/{yourname}/catkin_ws/map/{yourmapname}.yaml" />
