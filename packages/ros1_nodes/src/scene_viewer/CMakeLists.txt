cmake_minimum_required(VERSION 2.8.3)
project(scene_viewer)

find_package(catkin REQUIRED COMPONENTS
        rospy
        tf
        std_srvs
        std_msgs
        geometry_msgs
        tf2_msgs
        message_generation
        )

### Add messages
add_message_files(FILES
                  Caption.msg
                  Captions.msg
                  )

### Generate services and messages
generate_messages(DEPENDENCIES std_msgs std_srvs geometry_msgs tf2_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS rospy tf std_srvs std_msgs geometry_msgs tf2_msgs message_runtime
)

###########
## Build ##
###########
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
