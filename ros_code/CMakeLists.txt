cmake_minimum_required(VERSION 3.5.1)
project(f446_amr_shield_ros)
set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -pg -O2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}) 
find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)

SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_MULTITHREAD ON)
FIND_PACKAGE(Boost 1.42.0 REQUIRED COMPONENTS system thread regex)
IF(Boost_FOUND)
  INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
  LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
ENDIF(Boost_FOUND)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  DEPENDS Eigen
  CATKIN_DEPENDS roscpp
  INCLUDE_DIRS include
)

set(SRC ${SRC} 
  src/serial_node/serial_library_linux/serial_comm_linux.cpp
  src/serial_node/serial_library_linux/crc16.cpp
  src/serial_node/serial_comm_ros.cpp

  src/rostopic_node/rostopic_communicator.cpp

  src/serial_node/serial_comm_ros.cpp

  src/control_node/keyboard_controller.cpp
  src/control_node/keyboard_controller_position_yaw.cpp
  src/control_node/geometry_controller.cpp
  src/control_node/keyinput.cpp

  src/utility/timer.cpp
  src/utility/geometry_library.cpp
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )

# executables
add_executable(rostopic_node src/rostopic_node/main_rostopic_communicator.cpp ${SRC})
target_link_libraries (rostopic_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(serial_node src/serial_node/main_serial_node.cpp ${SRC})
target_link_libraries (serial_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_REGEX_LIBRARY})

add_executable(keyboard_controller_node src/control_node/main_keyboard_controller_position_yaw.cpp ${SRC})
target_link_libraries (keyboard_controller_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_REGEX_LIBRARY})

add_executable(geometry_controller_node src/control_node/main_geometry_controller.cpp ${SRC})
target_link_libraries (geometry_controller_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_REGEX_LIBRARY})

# add_executable(test_publisher_node src/test/test_publisher.cpp ${SRC})
# target_link_libraries (test_publisher_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

