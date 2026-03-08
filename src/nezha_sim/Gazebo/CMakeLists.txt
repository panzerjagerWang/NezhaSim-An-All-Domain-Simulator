cmake_minimum_required(VERSION 3.0.2)
project(nezha_gazebo)

find_package(catkin REQUIRED)
catkin_package()

# 把 models/ 与 worlds/ 整体安装到 share 目录
install(
  DIRECTORY models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

