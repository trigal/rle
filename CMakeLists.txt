cmake_minimum_required(VERSION 2.8.3)
project(road_layout_estimation)

#RelWithDebInfo
SET (CMAKE_BUILD_TYPE Release)
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -lrt -lnetica -lm")

find_package(catkin REQUIRED COMPONENTS     roscpp
                                            geometry_msgs
                                            cmake_modules
                                            message_generation
                                            std_msgs
                                            rosbag
)

find_package(PCL 1.8 REQUIRED )
find_package(Eigen REQUIRED)
find_package(message_generation)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} include include/netica)
link_directories(${PCL_LIBRARY_DIRS} include/netica)
add_definitions(${EIGEN_DEFINITIONS} ${PCL_DEFINITIONS})

add_definitions(-DEXEC_TEST=1)              #test number for transition matrix (Excel trans matrices)
add_definitions(-DMAKETRANSITIONSMATRICES)  #otherwise, create the matrices in code!
#add_definitions(-DSYNCMODE)  #send the sync for isis.player?
add_definitions(-DSAVEPATH="/home/cattaneod/") #path for output, temporary files and so on. This are deleted with deleteFiles()!
#add_definitions(-DGTFILE="catkin_ws/src/road_layout_estimation/bag/GT-A4FULL.txt")
#add_definitions(-DBAGFILE="/home/cattaneod/catkin_ws/src/road_layout_estimation/bag/mldhighway.a4.001_2017-12-18-12-38-52.bag")

#add_definitions(-DGTFILE="catkin_ws/src/road_layout_estimation/bag/ISIS_01.txt")
#add_definitions(-DBAGFILE="/home/cattaneod/catkin_ws/src/road_layout_estimation/bag/ISIS_01.bag")

add_definitions(-DGTFILE="catkin_ws/src/road_layout_estimation/bag/ISIS_01.txt")
add_definitions(-DBAGFILE="/home/cattaneod/catkin_ws/src/road_layout_estimation/bag/special.issue.spain.stereo.noClothoid.yesLines.bag")

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
#FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
#FOREACH(child ${children})
#  IF(IS_DIRECTORY ${child})
#    file(GLOB_RECURSE dir_files "${child}/*")
#    LIST(APPEND extra_files ${dir_files})
#  ENDIF()
#ENDFOREACH()
#add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

add_message_files(
  FILES
  msg_lineInfo.msg
  msg_lines.msg
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
add_executable(lane src/lane.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(road_layout_estimation_node road_layout_estimation_generate_messages_cpp)
add_dependencies(lane ${PROJECT_NAME}_gencfg)
add_dependencies(lane ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(lane  ${catkin_LIBRARIES} pthread netica)
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES src/model.cpp)
add_custom_target(netica SOURCES ${LibFiles})
