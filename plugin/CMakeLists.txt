cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)
project(CartConrolPlugin)

# Generate code from our own custom messages
# find_package catkin and genmsg, required for message generation macros
find_package(catkin REQUIRED COMPONENTS 
	dynamic_reconfigure
	roscpp
	rospy)
find_package(genmsg REQUIRED)
find_package(dynamic_reconfigure REQUIRED)




include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#find ros
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Enumerate our custom messages files
add_message_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} FILES
                                        msg/Velocity.msg
                                        msg/Position.msg
                                        msg/PathMsg.msg)
generate_messages(DEPENDENCIES geometry_msgs)

# Generate dynamic reconf
generate_dynamic_reconfigure_options(
  cfg/PIDconf.cfg
  #...
)

#find gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#find qt
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets)


#build libruary
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} ")
add_library(main SHARED main.cc
        kinematic/cartkinematic.cpp
        kinematic/cartkinematic.h
    )
add_dependencies(main ${PROJECT_NAME}_generate_messages ${PROJECT_NAME}_gencfg)
target_link_libraries(main  ${GAZEBO_LIBRARIES} 
							${Boost_LIBRARIES} 
							${roscpp_LIBRARIES}
							${catkin_LIBRARIES})



set(CMAKE_BUILD_TYPE Debug)
#build client
#add_executable(client client.cc)
add_executable(client 
                client/main.cpp
                client/mainwindow.cpp
                client/mainwindow.h
                client/mainwindow.ui
                client/resources.qrc
                client/cartcontrollerwidget.cpp
                client/cartcontrollerwidget.h
                client/rospublisher.cpp
                client/rospublisher.h
                client/cartpathsetter.cpp
                client/cartpathsetter.h
                client/commonheader.h
                client/cartpathabstract.cpp
                client/cartpathabstract.h
                client/cartpathgetter.cpp
                client/cartpathgetter.h
                kinematic/cartkinematic.cpp
                kinematic/cartkinematic.h
	)
target_link_libraries(client ${roscpp_LIBRARIES} Qt5::Widgets)
add_dependencies(client ${PROJECT_NAME}_generate_messages)

# Build cfg server
#dd_executable(drfServer  drcfServer.cpp)
#add_dependencies(drfServer ${PROJECT_NAME}_generate_messages ${PROJECT_NAME}_gencfg)
#target_link_libraries(drfServer ${Boost_LIBRARIES} ${roscpp_LIBRARIES} ${catkin_LIBRARIES})
