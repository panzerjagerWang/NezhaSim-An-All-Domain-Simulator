cmake_minimum_required(VERSION 3.0)

# 添加 ROS 路径到 CMAKE_PREFIX_PATH
set(CMAKE_PREFIX_PATH "/opt/ros/noetic;${CMAKE_PREFIX_PATH}")

message(STATUS "Testing mavlink detection...")

# 查找 mavlink - 模拟 Findmavlink.cmake 的逻辑
find_path(MAVLINK_INCLUDE_DIR
    NAMES mavlink/v1.0/mavlink_types.h mavlink/v2.0/mavlink_types.h
    PATH_SUFFIXES include
    PATHS /opt/ros/noetic/
)

if(MAVLINK_INCLUDE_DIR)
    message(STATUS "Found mavlink include dir: ${MAVLINK_INCLUDE_DIR}")
    
    # 检查版本
    if(EXISTS ${MAVLINK_INCLUDE_DIR}/mavlink/config.h)
        file(READ ${MAVLINK_INCLUDE_DIR}/mavlink/config.h MAVLINK_CONFIG_FILE)
        message(STATUS "Config file content: ${MAVLINK_CONFIG_FILE}")
        string(REGEX MATCH "#define MAVLINK_VERSION[ ]+\"(([0-9]+\\.)+[0-9]+)\""
            MAVLINK_VERSION_MATCH "${MAVLINK_CONFIG_FILE}")
        if(CMAKE_MATCH_1)
            message(STATUS "Found mavlink version: ${CMAKE_MATCH_1}")
        else()
            message(STATUS "Version regex didn't match, using default 2.0")
        endif()
    else()
        message(STATUS "Config file not found at: ${MAVLINK_INCLUDE_DIR}/mavlink/config.h")
    endif()
else()
    message(STATUS "mavlink include dir NOT found")
endif()
