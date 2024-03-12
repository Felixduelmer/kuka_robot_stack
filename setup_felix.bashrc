
export ROS_IP=172.31.1.150
export ROS_MASTER_URI=http://$ROS_IP:11311
source /home/felix/projects/catkin/catkin_ws/devel/setup.bash
export IMFUSION_PLUGIN_PATH=/home/felix/projects/catkin/catkin_ws/devel/lib

gnome-terminal -- /bin/sh -c "source /opt/ros/noetic/setup.bash; export ROS_IP=172.31.1.150; export ROS_MASTER_URI=http://$ROS_IP:11311; roscore"

sleep 2s

rosparam set /iiwa/toolName msot

ImFusionSuite

