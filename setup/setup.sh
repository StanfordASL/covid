
mkdir -p ~/src/
cd ~/src
git clone https://www.github.com/StanfordASL/Firmware
cd Firmware
no_sim=1 make px4_sitl_default gazebo

source ~/catkin_ws/src/covid/ubuntu_sim_ros_melodic.sh



