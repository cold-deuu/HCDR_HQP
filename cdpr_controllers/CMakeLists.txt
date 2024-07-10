cmake_minimum_required(VERSION 3.0.2)
project(cdpr_controllers)


find_package(Boost REQUIRED COMPONENTS system)
find_package(pinocchio REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rci_cdpr_controller REQUIRED)
find_package(cdpr REQUIRED)
find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cdpr
  roscpp
  sensor_msgs
  geometry_msgs
  actionlib_msgs
  actionlib
  rci_cdpr_controller
)

add_action_files(DIRECTORY action FILES
  SE3.action PlatformSE3.action JointPosture.action FrankaSE3.action
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs actionlib_msgs)


set(CMAKE_CXX_STANDARD 14)
###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME} 
 CATKIN_DEPENDS 
 controller_interface
 pluginlib
 roscpp
 sensor_msgs
 std_msgs
)



###########
## Build ##
###########

include_directories(
    include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${rci_cdpr_controller_INCLUDE_DIRS}
  ${cdpr_INCLUDE_DIRS}

)

install(PROGRAMS
	
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)

# Library
SET(${PROJECT_NAME}_HEADERS
  #Simulator
  include/cdpr_controllers/controller/controller.hpp
  include/cdpr_controllers/simulator/simulator.hpp

  #Server
  include/cdpr_controllers/server/joint_posture_server.hpp
  include/cdpr_controllers/server/wholebody_server.hpp
  include/cdpr_controllers/server/platform_se3_server.hpp
  include/cdpr_controllers/server/franka_se3_server.hpp
  include/cdpr_controllers/server/action_server_base.hpp

  #Task
  include/cdpr_controllers/task/SE3_Track.hpp
  include/cdpr_controllers/task/Pf_ori_fix.hpp
  include/cdpr_controllers/task/Pf_pos_fix.hpp
  include/cdpr_controllers/task/Franka_pos_fix.hpp

  )

SET(${PROJECT_NAME}_SOURCES
  #Controller
  src/controller/controller.cpp

  #Server
  src/server/wholebody_server.cpp
  src/server/platform_se3_server.cpp
  src/server/franka_se3_server.cpp
  src/server/joint_posture_server.cpp

  #Task
  src/task/SE3_Track.cpp
  src/task/Pf_ori_fix.cpp
  src/task/Pf_pos_fix.cpp
  src/task/Franka_pos_fix.cpp

)

SET(CMAKE_BUILD_TYPE "Release")

ADD_LIBRARY(${PROJECT_NAME} SHARED  ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include> )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC ${rci_cdpr_controller_LIBRARIES} ${cdpr_LIBRARIES} pinocchio::pinocchio eiquadprog::eiquadprog -lpthread)

add_executable(${PROJECT_NAME}_action src/simulator/simulator.cpp)
add_dependencies(${PROJECT_NAME}_action ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_action ${PROJECT_NAME} eiquadprog::eiquadprog  pinocchio::pinocchio)
target_link_libraries(${PROJECT_NAME}_action ${catkin_LIBRARIES})

