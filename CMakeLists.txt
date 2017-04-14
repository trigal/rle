cmake_minimum_required(VERSION 2.8.3)
project(road_layout_estimation)

#RelWithDebInfo
#SET (CMAKE_BUILD_TYPE Debug)
#SET (ROS_BUILD_TYPE Debug)
#SET (CMAKE_CXX_FLAGS "-O0 -std=c++11") #RelWithDebInfo Release
SET (CMAKE_BUILD_TYPE Release)
SET (ROS_BUILD_TYPE Release)
SET (CMAKE_CXX_FLAGS "-O3 -std=c++11")

add_definitions (-DEXEC_TEST=12)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(message_generation)
find_package(catkin REQUIRED COMPONENTS     roscpp
                                            tf
                                            nav_msgs
                                            geometry_msgs
                                            dynamic_reconfigure
                                            cmake_modules
                                            message_generation
                                            std_msgs
                                            road_lane_detection
                                            ira_open_street_map
                                            nodelet
)

find_package( PCL 1.8 REQUIRED )
find_package(Eigen REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${EIGEN_DEFINITIONS} ${PCL_DEFINITIONS})

generate_dynamic_reconfigure_options(
    cfg/road_layout_estimation.cfg
    cfg/visual_odometry_fake.cfg
    cfg/visual_odometry_nvm.cfg
    cfg/visual_odometry_test.cfg
)

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

#find_package( PCL REQUIRED )
#include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  msg_roadState.msg
  msg_lineInfo.msg
  msg_lines.msg
  msg_debugInformation.msg
)

add_service_files(
  FILES
  getAllParticlesLatLon.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES road_layout_estimation
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(road_layout_estimation
#   src/${PROJECT_NAME}/road_layout_estimation.cpp
# )

## Declare cpp executables
add_executable(visual_odometry_fake 
	src/visual_odometry_fake.cpp 
        src/particle/State6DOF.cpp
        src/Utils.cpp
)

add_executable(roadStateComponent_fake
        src/roadStateComponent_fake.cpp
        src/particle/State6DOF.cpp
        src/Utils.cpp
)

add_executable(visual_odometry_test
        src/visual_odometry_test.cpp
        src/particle/State6DOF.cpp
        src/Utils.cpp
)

add_executable(road_layout_estimation
        src/road_layout_estimation.cpp
        src/Utils.cpp
	src/LayoutManager.cpp 
        src/MeasurementModel.cpp
	src/particle/MotionModel.cpp
	src/particle/Particle.cpp
        src/particle/LayoutComponent.cpp
        src/particle/LayoutComponent_Building.cpp
        src/particle/LayoutComponent_Crossing.cpp
        src/particle/LayoutComponent_RoadLane.cpp
        src/particle/State6DOF.cpp
        src/eigenmultivariatenormal.hpp
        src/particle/LayoutComponent_RoadState.cpp
        src/particle/LayoutComponent_OSMDistance.cpp
        src/Facade.hpp
        src/Edge.hpp
)

add_executable(visual_odometry_nvm
        src/visual_odometry_nvm.cpp
        src/particle/State6DOF.cpp
        src/Utils.cpp
)

add_executable(fake_callback
        src/fake_callback.cpp
)
add_executable(lane
        src/lane.cpp
)

add_executable(shared_test
        src/shared_test.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(road_layout_estimation_node road_layout_estimation_generate_messages_cpp)

#add_dependencies(road_layout_estimation ${catkin_EXPORTED_TARGETS})

add_dependencies(fake_callback ${PROJECT_NAME}_gencfg)
add_dependencies(fake_callback ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(lane ${PROJECT_NAME}_gencfg)
add_dependencies(lane ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(road_layout_estimation ${PROJECT_NAME}_gencfg)
add_dependencies(road_layout_estimation ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(roadStateComponent_fake ${PROJECT_NAME}_gencfg)
add_dependencies(roadStateComponent_fake ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(visual_odometry_fake ${PROJECT_NAME}_gencfg)
add_dependencies(visual_odometry_nvm ${PROJECT_NAME}_gencfg)
add_dependencies(visual_odometry_test ${PROJECT_NAME}_gencfgf)
add_dependencies(roadStateComponent_fake ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(road_layout_estimation
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

target_link_libraries(visual_odometry_fake
  ${catkin_LIBRARIES}
)

target_link_libraries(roadStateComponent_fake
  ${catkin_LIBRARIES}
)

target_link_libraries(visual_odometry_test
  ${catkin_LIBRARIES}
)

target_link_libraries(visual_odometry_nvm
#   ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(fake_callback
  ${catkin_LIBRARIES}
)
target_link_libraries(lane
  ${catkin_LIBRARIES}
)
