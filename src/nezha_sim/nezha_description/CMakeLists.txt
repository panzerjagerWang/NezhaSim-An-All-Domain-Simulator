#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#
cmake_minimum_required(VERSION 3.0.2)
project(nezha_description)

find_package(catkin REQUIRED COMPONENTS xacro)

catkin_package()

# Install models / URDF / Xacro / meshes / textures / launch and other resources
# (add or remove entries to match your actual directory layout)
install(DIRECTORY
  nezha_mini/
  nezha_rocket/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN ".git" EXCLUDE
  PATTERN ".svn" EXCLUDE
)

