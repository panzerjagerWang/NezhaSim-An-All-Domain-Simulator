#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#
cmake_minimum_required(VERSION 3.0.2)
project(nezha_gazebo)

find_package(catkin REQUIRED)
catkin_package()

# Install the entire models/ and worlds/ directories into the share directory
install(
  DIRECTORY models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

