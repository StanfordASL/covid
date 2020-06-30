
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src/covid/Firmware
echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/catkin_ws/src/covid/Firmware" >> ~/.bashrc
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src/covid/Firmware/Tools/sitl_gazebo
echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/catkin_ws/src/covid/Firmware/Tools/sitl_gazebo" >> ~/.bashrc
source ~/catkin_ws/src/covid/Firmware/Tools/setup_gazebo.bash ~/catkin_ws/src/covid/Firmware ~/catkin_ws/src/covid/Firmware/build/px4_sitl_default
echo "source ~/catkin_ws/src/covid/Firmware/Tools/setup_gazebo.bash ~/catkin_ws/src/covid/Firmware ~/catkin_ws/src/covid/Firmware/build/px4_sitl_default>> ~/.bashrc


