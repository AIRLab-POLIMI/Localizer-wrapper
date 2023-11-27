cmake_minimum_required(VERSION 3.5)
project(localizer_wrapper)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_library(LOTS_CORE_IO_LIBS REQUIRED
        NAMES lots_core_io_lib
        HINTS "${PROJECT_SOURCE_DIR}/../../LOTS-CORE/lib"
        )

find_library(LOTS_CORE_UTILS_LIBS REQUIRED
        NAMES lots_core_utils_lib
        HINTS "${PROJECT_SOURCE_DIR}/../../LOTS-CORE/lib"
        )

find_library(LOTS_LOCALIZER_LIBS REQUIRED
    NAMES localization_lib
    HINTS "${PROJECT_SOURCE_DIR}/../../LOTS-Localization/lib"
)

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.12.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(GTSAM 4 REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

include_directories(${PROJECT_SOURCE_DIR}/../../LOTS-CORE/include
                    ${PROJECT_SOURCE_DIR}/../../LOTS-CORE/src
                    ${PROJECT_SOURCE_DIR}/../../LOTS-Localization/
                    )

find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
    include
)
    
add_executable(artslam_localizer
    src/ArtslamLocalizer.cpp
)
ament_target_dependencies(artslam_localizer
    rclcpp
    sensor_msgs
    nav_msgs
    pcl_conversions
    pcl_ros
    pcl_msgs
    tf2_ros
    geometry_msgs
    tf2
)
target_link_libraries(artslam_localizer
    gtsam
    ${LOTS_LOCALIZER_LIBS}
    ${LOTS_CORE_UTILS_LIBS}
    ${PCL_LIBRARIES}
)

install(TARGETS
    artslam_localizer
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
