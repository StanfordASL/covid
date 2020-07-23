pip3 install --user packaging

cd ~/catkin_ws/src/covid/Firmware
make clean
make px4_sitl_default gazebo DONT_RUN=1
cd ..


source ~/catkin_ws/src/covid/setup/realsense_setup.sh
source ~/catkin_ws/src/covid/setup/ubuntu_sim_ros_melodic.sh

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/src/covid/setup/ros_package.sh" >> ~/.bashrc

source ~/catkin_ws/devel/setup.bash
source ~/catkin_ws/src/covid/setup/ros_package.sh

echo ""
echo "Wow it is a miracle, everything worked!"



