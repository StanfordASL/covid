pip3 install --user packaging

cd ~/catkin_ws/src/covid/Firmware
make clean
make px4_sitl_default gazebo DONT_RUN=1
cd ..

mkdir temp && cd temp
git clone https://github.com/danfis/libccd
cd libccd/src
make
sudo make install
cd ..
git clone https://github.com/flexible-collision-library/fcl
cd fcl
mkdir build && cd build
cmake ..
make 
sudo make install
cd ..
sudo apt-get update
sudo apt-get install ros-melodic-ompl*
wget https://ompl.kavrakilab.org/install-ompl-ubuntu.sh
chmod u+x install-ompl-ubuntu.sh
sudo source install-ompl-ubuntu.sh
cd ..
sudo rm -rf temp



source ~/catkin_ws/src/covid/setup/realsense_setup.sh
source ~/catkin_ws/src/covid/setup/ubuntu_sim_ros_melodic.sh

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/src/covid/setup/ros_package.sh" >> ~/.bashrc

source ~/catkin_ws/devel/setup.bash
source ~/catkin_ws/src/covid/setup/ros_package.sh

echo ""
echo "Wow it is a miracle, everything worked!"



