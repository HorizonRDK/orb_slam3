cmake_minimum_required(VERSION 3.5)
project(Examples_ROS2)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -w -O3 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -O3 ")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ")


## Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0x)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
#
#LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
#

find_package(OpenCV)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 REQUIRED)
find_package(realsense2)


# find ROS2_dependencies
find_package(console_bridge 1.0.1 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

message("console_bridge dir")
message(${console_bridge_INCLUDE_DIRS})
message(${console_bridge_VERSION})

get_filename_component(SECOND_FOLDER_ABSOLUTE ${PROJECT_SOURCE_DIR} DIRECTORY)

include_directories(
${SECOND_FOLDER_ABSOLUTE}
${SECOND_FOLDER_ABSOLUTE}/include
${SECOND_FOLDER_ABSOLUTE}/include/CameraModels
${SECOND_FOLDER_ABSOLUTE}/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS}
)



set(LIBS
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${SECOND_FOLDER_ABSOLUTE}/lib/libDBoW2.so
        ${SECOND_FOLDER_ABSOLUTE}/lib/libg2o.so
        ${SECOND_FOLDER_ABSOLUTE}/lib/libORB_SLAM3.so
        -lboost_system
        )

#build ROS2 examples
add_executable(stereo_inertial src/ros2_stereo_inertial.cpp)
target_link_libraries(stereo_inertial ${LIBS})
ament_target_dependencies(stereo_inertial rclcpp
                                          sensor_msgs
                                          nav_msgs
                                          rclcpp_components
                                          cv_bridge
                                          tf2
                                          tf2_ros
                                          console_bridge)
install(TARGETS stereo_inertial DESTINATION lib/${PROJECT_NAME})

#stereo
add_executable(stereo src/ros2_stereo.cpp)
target_link_libraries(stereo ${LIBS})
ament_target_dependencies(stereo rclcpp
                                 sensor_msgs
                                 nav_msgs
                                 rclcpp_components
                                 cv_bridge
                                 tf2
                                 tf2_ros
                                 console_bridge)
install(TARGETS stereo DESTINATION lib/${PROJECT_NAME})

#rgbd_inertial
add_executable(rgbd_inertial src/ros2_rgbd_inertial.cpp)
target_link_libraries(rgbd_inertial ${LIBS})
ament_target_dependencies(rgbd_inertial  rclcpp
                                        sensor_msgs
                                        nav_msgs
                                        rclcpp_components
                                        cv_bridge
                                        tf2
                                        tf2_ros
                                        console_bridge)
install(TARGETS rgbd_inertial DESTINATION lib/${PROJECT_NAME})

#rgbd
add_executable(rgbd src/ros2_rgbd.cpp)
target_link_libraries(rgbd ${LIBS})
ament_target_dependencies(rgbd  rclcpp
                                sensor_msgs
                                nav_msgs
                                rclcpp_components
                                cv_bridge
                                tf2
                                tf2_ros
                                console_bridge)
install(TARGETS rgbd DESTINATION lib/${PROJECT_NAME})

#mono_inertial
add_executable(mono_inertial src/ros2_mono_inertial.cpp)
target_link_libraries(mono_inertial ${LIBS})
ament_target_dependencies(mono_inertial rclcpp
                                        sensor_msgs
                                        nav_msgs
                                        rclcpp_components
                                        cv_bridge
                                        tf2
                                        tf2_ros
                                        console_bridge)
install(TARGETS mono_inertial DESTINATION lib/${PROJECT_NAME})

#mono
add_executable(mono src/ros2_mono.cpp)
target_link_libraries(mono ${LIBS})
ament_target_dependencies(mono  rclcpp
                                sensor_msgs
                                nav_msgs
                                rclcpp_components
                                cv_bridge
                                tf2
                                tf2_ros
                                console_bridge)
install(TARGETS mono DESTINATION lib/${PROJECT_NAME})



file(INSTALL ${SECOND_FOLDER_ABSOLUTE}/lib/libDBoW2.so DESTINATION ${PROJECT_SOURCE_DIR}/install/lib)
file(INSTALL ${SECOND_FOLDER_ABSOLUTE}/lib/libg2o.so DESTINATION ${PROJECT_SOURCE_DIR}/install/lib)
file(INSTALL ${SECOND_FOLDER_ABSOLUTE}/lib/libORB_SLAM3.so DESTINATION ${PROJECT_SOURCE_DIR}/install/lib)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

