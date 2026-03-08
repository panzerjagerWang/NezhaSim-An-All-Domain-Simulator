cmake_minimum_required(VERSION 3.0.2)
project(nezha_description)

find_package(catkin REQUIRED COMPONENTS xacro)

catkin_package()

# 安装模型/URDF/Xacro/网格/贴图/launch 等资源
# （根据你的实际目录结构可增减）
install(DIRECTORY
  nezha_mini/
  nezha_rocket/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN ".git" EXCLUDE
  PATTERN ".svn" EXCLUDE
)

