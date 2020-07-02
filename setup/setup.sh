pip3 install --user packaging
source ~/catkin_ws/src/covid/setup/realsense_setup.sh
source ~/catkin_ws/src/covid/setup/ubuntu_sim_ros_melodic.sh
cd ~/catkin_ws/src/covid/Firmware
make clean
make px4_sitl_default gazebo DONT_RUN=1
cd ..

source ~/catkin_ws/src/covid/setup/ros_package.sh

echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/catkin_ws/src/covid/Firmware" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/catkin_ws/src/covid/Firmware/Tools/sitl_gazebo" >> ~/.bashrc
echo "source ~/catkin_ws/src/covid/Firmware/Tools/setup_gazebo.bash ~/catkin_ws/src/covid/Firmware ~/catkin_ws/src/covid/Firmware/build/px4_sitl_default >/dev/null" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.sh" >> ~/.bashrc

source ~/catkin_ws/devel/setup.bash

echo ""
echo "Wow it is a miracle, everything worked!"



