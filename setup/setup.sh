pip3 install --user packaging

mkdir -p ~/src/
git clone https://www.github.com/StanfordASL/Firmware ~/src/Firmware
cd ~/src/Firmware
git checkout covid --recurse-submodules
make px4_sitl_default gazebo DONT_RUN=1

source ~/catkin_ws/src/covid/setup/realsense_setup.sh

source ~/catkin_ws/src/covid/setup/ubuntu_sim_ros_melodic.sh
cd ~/catkin_ws/src/covid



