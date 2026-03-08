cmake_minimum_required(VERSION 3.10)
project(nezha_plugins LANGUAGES CXX)
file(GLOB ASV_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")

# Drop tests and plugin demos from the sources:
list(FILTER ASV_SRCS EXCLUDE REGEX "_TEST\\.cc$")
list(FILTER ASV_SRCS EXCLUDE REGEX "/GzTestRunner\\.cc$")
list(FILTER ASV_SRCS EXCLUDE REGEX "/TestRunner\\.cc$")
list(FILTER ASV_SRCS EXCLUDE REGEX "/HydrodynamicsPlugin\\.cc$")
list(FILTER ASV_SRCS EXCLUDE REGEX "/WavefieldModelPlugin\\.cc$")
list(FILTER ASV_SRCS EXCLUDE REGEX "/WavefieldVisualPlugin\\.cc$")

# Then use ASV_SRCS when creating the target, e.g.:
# add_library(asv_wave_sim_gazebo_plugins SHARED ${ASV_SRCS})
add_compile_definitions(TBB_SUPPRESS_DEPRECATED_MESSAGES=1)
add_compile_options(-Wno-deprecated-declarations)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)   

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Release Debug RelWithDebInfo" FORCE)
endif()

add_compile_options(
  -Wall -Wextra -Wpedantic
  $<$<CONFIG:Release>:-O3>
)


find_package(gazebo 11 REQUIRED)
find_package(Boost  REQUIRED COMPONENTS system filesystem thread)
find_package(TBB REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_msgs
  gazebo_ros
  rotors_comm
  rotors_gazebo_plugins
  uuv_gazebo_ros_plugins_msgs
  uuv_world_ros_plugins_msgs
  mav_msgs
  message_generation
)
find_package(Protobuf REQUIRED)
set(PROTOBUF_IMPORT_DIRS "")
set(GAZEBO_MSG_INCLUDE_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    list(APPEND PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
    list(APPEND GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()
message(STATUS "PROTOBUF_IMPORT_DIRS = ${PROTOBUF_IMPORT_DIRS}")
include_directories(${GAZEBO_MSG_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})
find_package(nlohmann_json 3.2.0 REQUIRED)
add_service_files(FILES
  GetPhaseSample.srv
  HydrodynamicsForces.srv
  SetWaveParameters.srv
)
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs  
)

catkin_package(
  LIBRARIES     nezha_dynamics
  INCLUDE_DIRS  include
  CATKIN_DEPENDS 
    roscpp  
    gazebo_msgs 
    gazebo_ros 
    uuv_gazebo_ros_plugins_msgs 
    uuv_world_ros_plugins_msgs 
    mav_msgs 
    message_runtime          
    rotors_comm 
    rotors_gazebo_plugins
)

set(THIRD_PARTY_DIR ${PROJECT_SOURCE_DIR}/../third_party)

set(NEZHA_INCLUDE_DIRS
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${THIRD_PARTY_DIR}/rotors_simulator/rotors_gazebo_plugins/include
  ${THIRD_PARTY_DIR}/asv_wave_sim/asv_wave_sim_gazebo_plugins/include
  ${THIRD_PARTY_DIR}/uuv_simulator-noetic/uuv_gazebo_plugins/uuv_gazebo_plugins/include
  ${THIRD_PARTY_DIR}/uuv_simulator-noetic/uuv_gazebo_plugins/uuv_gazebo_ros_plugins/include
  ${THIRD_PARTY_DIR}/uuv_simulator-noetic/uuv_gazebo_plugins/uuv_gazebo_ros_plugins_msgs/include
)


set(LIB_OUTPUT_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})

include(GNUInstallDirs)
if(NOT DEFINED GAZEBO_PLUGIN_INSTALL_DIR)
  set(GAZEBO_PLUGIN_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
endif()
add_library(nezha_dynamics STATIC
  src/nezha_underwaterHydrodynamicsPlugin.cc
  src/nuuv_buoyantObject.cc 
)
target_include_directories(nezha_dynamics PUBLIC ${NEZHA_INCLUDE_DIRS})
target_link_libraries(nezha_dynamics PUBLIC ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} TBB::tbb)
set_target_properties(nezha_dynamics PROPERTIES
  POSITION_INDEPENDENT_CODE ON        
  ARCHIVE_OUTPUT_DIRECTORY  ${LIB_OUTPUT_DIR}
)


install(TARGETS nezha_dynamics
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})


function(link_plugin_common target)
  set(EXTRA_LIBS ${ARGN})  
  target_include_directories(${target} PUBLIC ${NEZHA_INCLUDE_DIRS})
  target_link_libraries(${target}
    PUBLIC ${EXTRA_LIBS} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} 
  )
  #target_compile_definitions(${target} PRIVATE GZ_PLUGIN_VISIBLE)
  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${LIB_OUTPUT_DIR}
  )
  install(TARGETS ${target}
          LIBRARY DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
endfunction()
# 4.5 ASV wave-sim core (static + PIC, exclude *_TEST.cc and ASV plugins)
set(ASV_DIR ${THIRD_PARTY_DIR}/asv_wave_sim/asv_wave_sim_gazebo_plugins)
set(ASV_SRC ${ASV_DIR}/src)

add_library(asv_wave_core STATIC
  ${ASV_SRC}/Utilities.cc
  ${ASV_SRC}/Convert.cc
  ${ASV_SRC}/Grid.cc
  ${ASV_SRC}/Wavefield.cc
  ${ASV_SRC}/WavefieldEntity.cc
  ${ASV_SRC}/Geometry.cc
  ${ASV_SRC}/MeshTools.cc
  ${ASV_SRC}/Algorithm.cc
  ${ASV_SRC}/Physics.cc
  ${ASV_SRC}/Gazebo.cc
  ${ASV_SRC}/PhysicalConstants.cc
)
set_target_properties(asv_wave_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(asv_wave_core PUBLIC
  ${ASV_DIR}/include
  ${NEZHA_INCLUDE_DIRS})

# Quiet some third-party warnings (optional)
target_compile_definitions(asv_wave_core PRIVATE TBB_SUPPRESS_DEPRECATED_MESSAGES=1)
target_compile_options(asv_wave_core PRIVATE -Wno-deprecated-declarations)

target_link_libraries(asv_wave_core PUBLIC
  ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})


# 5.1 HydrodynamicsPlugin
add_library(nezha_surface_plugin SHARED
  src/nezha_surfacePlugin.cc
  src/nasv_physics.cc
)
link_plugin_common(nezha_surface_plugin nezha_dynamics asv_wave_core)

# 5.2 Motor Model
add_library(nezha_motor_model SHARED
  src/nezha_motorModel.cpp
  src/nezha_getWavefield.cc        # ← 添加这个
  src/nasv_waveField.cc            # ← 如果也需要的话
)


link_plugin_common(nezha_motor_model)
target_link_libraries(nezha_motor_model PUBLIC 
  nezha_dynamics
  asv_wave_core 
)


set(ROTORS_ROOT     ${THIRD_PARTY_DIR}/rotors_simulator/rotors_gazebo_plugins)
set(ROTORS_MSGS_DIR ${ROTORS_ROOT}/msgs)


file(GLOB ROTORS_PB_SRCS "${ROTORS_MSGS_DIR}/*.pb.cc")

if(ROTORS_PB_SRCS)
  message(STATUS "[nezha_motor_model] Using pre-generated pb sources in: ${ROTORS_MSGS_DIR}")
  target_sources(nezha_motor_model PRIVATE ${ROTORS_PB_SRCS})

  target_include_directories(nezha_motor_model PRIVATE "${ROTORS_MSGS_DIR}")

else()
  message(STATUS "[nezha_motor_model] Generating pb sources from ALL .proto in: ${ROTORS_MSGS_DIR}")

  file(GLOB ROTORS_PROTO_FILES "${ROTORS_MSGS_DIR}/*.proto")
  if(NOT ROTORS_PROTO_FILES)
    message(FATAL_ERROR "No .proto files found in ${ROTORS_MSGS_DIR}")
  endif()

  if(NOT Protobuf_FOUND)
    find_package(Protobuf REQUIRED)
  endif()

list(APPEND PROTOBUF_IMPORT_DIRS "${ROTORS_MSGS_DIR}")
list(REMOVE_DUPLICATES PROTOBUF_IMPORT_DIRS)
message(STATUS "[nezha_motor_model] PROTOBUF_IMPORT_DIRS = ${PROTOBUF_IMPORT_DIRS}")

  set(PROTOBUF_GENERATE_CPP_APPEND_PATH TRUE) 
  PROTOBUF_GENERATE_CPP(ROTORS_PROTO_SRCS ROTORS_PROTO_HDRS ${ROTORS_PROTO_FILES})

  target_sources(nezha_motor_model PRIVATE ${ROTORS_PROTO_SRCS})

  target_include_directories(nezha_motor_model
    PRIVATE
      "${CMAKE_CURRENT_BINARY_DIR}"   
      "${ROTORS_MSGS_DIR}"           
  )

endif()


if (TARGET protobuf::libprotobuf)
  target_link_libraries(nezha_motor_model PRIVATE protobuf::libprotobuf)
else()
  target_link_libraries(nezha_motor_model PRIVATE ${PROTOBUF_LIBRARIES})
endif()


target_link_options(nezha_motor_model PRIVATE -Wl,--no-undefined)


# add_dependencies(nezha_motor_model gz_std_msgs ${catkin_EXPORTED_TARGETS})
add_dependencies(nezha_motor_model ${catkin_EXPORTED_TARGETS})


# 5.3 Robot State Plugin
add_library(nezha_robot_state_plugin SHARED
  src/nezha_robotStatePlugin.cc
)
link_plugin_common(nezha_robot_state_plugin)
target_link_libraries(nezha_robot_state_plugin PUBLIC nezha_dynamics)

# 5.4 Terrain Detector
add_library(nezha_terrain_detector_plugin SHARED
  src/nezha_terrainDetectorPlugin.cpp
)
link_plugin_common(nezha_terrain_detector_plugin)
target_link_libraries(nezha_terrain_detector_plugin PUBLIC nezha_dynamics)

# 5.5 Wheel Efficiency Plugin (带波浪场支持)
add_library(nezha_wheel_efficiency_plugin SHARED
  src/nezha_wheelEfficiency.cpp
  src/nezha_getWavefield.cc      
  src/nasv_waveField.cc         
)
link_plugin_common(nezha_wheel_efficiency_plugin)
target_link_libraries(nezha_wheel_efficiency_plugin 
  PUBLIC 
    nezha_dynamics
    asv_wave_core
)
add_dependencies(nezha_wheel_efficiency_plugin ${catkin_EXPORTED_TARGETS})
# 5.6 Wavefield Model
add_library(nezha_wavefieldmodel_plugin SHARED
  src/nasv_wavefieldModelPlugin.cc
  src/nezha_getWavefield.cc         
  src/nasv_waveField.cc
)
link_plugin_common(nezha_wavefieldmodel_plugin)
target_link_libraries(nezha_wavefieldmodel_plugin
  PUBLIC nezha_dynamics asv_wave_core
  PRIVATE gazebo_transport ${GAZEBO_LIBRARIES})

# 5.7 Under-water Object
add_library(nezha_underwater_plugin SHARED
  src/nezha_underwaterPlugin.cc
)
link_plugin_common(nezha_underwater_plugin)
target_link_libraries(nezha_underwater_plugin PUBLIC nezha_dynamics)


add_library(nezha_wave_probe_plugin SHARED
  src/nezha_waveProbePlugin.cc
  src/nezha_getWavefield.cc
  src/nasv_waveField.cc
)


add_dependencies(nezha_wave_probe_plugin 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}  
  ${catkin_EXPORTED_TARGETS}         
)

target_include_directories(nezha_wave_probe_plugin
  PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include 
    ${NEZHA_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/asv_wave_sim/asv_wave_sim_gazebo_plugins/include
)

target_link_libraries(nezha_wave_probe_plugin
  PRIVATE
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    nlohmann_json::nlohmann_json
)


# 5.5 Transmedia DragPlugin
add_library(nezha_transmedia_drag_plugin SHARED src/nezha_transmediaDragPlugin.cc)
add_dependencies(nezha_transmedia_drag_plugin ${catkin_EXPORTED_TARGETS})
target_include_directories(nezha_transmedia_drag_plugin
  PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include   
)
target_link_libraries(nezha_transmedia_drag_plugin
  PRIVATE
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    nlohmann_json::nlohmann_json
)

add_library(nezha_buoyant_object_plugin SHARED
  src/nuuv_buoyantObject.cc      
)
link_plugin_common(nezha_buoyant_object_plugin)
target_link_libraries(nezha_buoyant_object_plugin PUBLIC nezha_dynamics)


add_library(nezha_wavefield_plugin SHARED
  src/nasv_waveField.cc
  src/nezha_getWavefield.cc        
)
link_plugin_common(nezha_wavefield_plugin nezha_dynamics asv_wave_core )

target_include_directories(nezha_wavefield_plugin
  PRIVATE
    ${GAZEBO_INCLUDE_DIRS}
)


# 5.9 Phase Switch Plugin
# 5.9 Phase Switch Plugin
add_library(nezha_phase_switch_plugin SHARED
  src/nezha_phaseSwitchPlugin.cc
)

link_plugin_common(nezha_phase_switch_plugin)
target_link_libraries(nezha_phase_switch_plugin PUBLIC nezha_dynamics)
add_dependencies(nezha_phase_switch_plugin ${catkin_EXPORTED_TARGETS})


install(TARGETS nezha_wave_probe_plugin
                nezha_transmedia_drag_plugin
                nezha_buoyant_object_plugin
                nezha_wavefield_plugin
                nezha_phase_switch_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


message(STATUS "")
message(STATUS "----------- NEZHA PLUGINS (multi .so) -----------")
message(STATUS "Build type          : ${CMAKE_BUILD_TYPE}")
message(STATUS "Gazebo include dirs : ${GAZEBO_INCLUDE_DIRS}")
message(STATUS "Install dir (.so)   : ${GAZEBO_PLUGIN_INSTALL_DIR}")
message(STATUS "--------------------------------------------------")

