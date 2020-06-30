pip3 install --user packaging
source ~/catkin_ws/src/covid/setup/realsense_setup.sh
source ~/catkin_ws/src/covid/setup/ubuntu_sim_ros_melodic.sh
cd ~/catkin_ws/src/covid/Firmware
make px4_sitl_default gazebo DONT_RUN=1
cd ..

source ~/catkin_ws/src/covid/setup/ros_package.sh
source ~/catkin_ws/devel/setup.sh

echo "/nWow it is a miracle, everything worked!"



