sudo apt-get update
source ~/catkin_ws/src/covid/setup/ubuntu_sim_ros_melodic.sh
sudo rosdep init
rosdep update
cd ~/catkin_ws/src/covid/Firmware
make clean
make px4_sitl_default gazebo -j8 DONT_RUN=1
cd ..

mkdir temp && cd temp

#Libccd
git clone https://github.com/danfis/libccd
cd libccd
mkdir build && cd build
cmake -G "Unix Makefiles" ..
make && sudo make install
cd ..
sudo apt-get install ros-melodic-octomap
#Flexible Collision Library
git clone https://github.com/StanfordASL/fcl
cd fcl
mkdir build && cd build
cmake -DFCL_USE_X64_SSE=OFF ..
make -j$(nproc)
sudo make install
cd ..

#OMPL
wget https://ompl.kavrakilab.org/install-ompl-ubuntu.sh
chmod u+x install-ompl-ubuntu.sh
sudo bash install-ompl-ubuntu.sh
cd ..

sudo rm -rf temp


#source ~/catkin_ws/src/covid/setup/realsense_setup.sh
pip install future
sudo apt-get install ros-melodic-geographic-msgs
sudo apt-get install ros-melodic-octomap-ros
sudo apt-get install libgeographic-dev
sudo apt-get install libxslt-dev
sudo apt-get install libncurses5-dev
pip install lxml

## Build!
catkin clean
catkin build
## Re-source environment to reflect new packages/build environment

catkin_ws_source="source ~/catkin_ws/devel/setup.bash"
if grep -Fxq "$catkin_ws_source" ~/.bashrc; then echo ROS catkin_ws setup.bash already in .bashrc;
else echo "$catkin_ws_source" >> ~/.bashrc; fi
eval $catkin_ws_source
echo "source ~/catkin_ws/src/covid/setup/ros_package.sh" >> ~/.bashrc
source ~/catkin_ws/src/covid/setup/ros_package.sh


