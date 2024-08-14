cmake_minimum_required(VERSION 2.8.3)
project(tb_waiter)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    actionlib message_generation cv_bridge image_transport message_filters nav_msgs roscpp rospy 
    sensor_msgs std_msgs geometry_msgs kobuki_node openni_camera tf kobuki_gazebo_plugins xacro
)
find_package(OpenCV REQUIRED)

include_directories(include 
		    ${catkin_INCLUDE_DIRS} 
		    ${Boost_INCLUDE_DIRS})

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ContractNetMsg.msg
   DynamicRoleAssigment.msg
   DynamicRoleInfo.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  robPosInfo.srv
  robPlanDist.srv
  robResetPos.srv
  cvMarkRec.srv
)

## Generate actions in the 'action' folder
add_action_files(
   FILES
   TurtlebotMove.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   nav_msgs   
   std_msgs
   actionlib_msgs
   geometry_msgs
 )
 
# 
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES kobuki_auto_docking_ros kobuki_auto_docking_nodelet
   CATKIN_DEPENDS  actionlib message_generation cv_bridge image_transport message_filters nav_msgs roscpp rospy 
   		   sensor_msgs std_msgs geometry_msgs kobuki_node openni_camera tf kobuki_gazebo_plugins xacro
)
 
 # libraries
add_library(KobukiBase src/turtlebot/kobuki_base.cpp)
target_link_libraries(KobukiBase ${catkin_LIBRARIES})
add_dependencies(KobukiBase tb_waiter_gencpp)

add_library(KobukiCommand src/turtlebot/kobuki_command.cpp)
target_link_libraries(KobukiCommand ${catkin_LIBRARIES})
add_dependencies(KobukiCommand tb_waiter_gencpp)

add_library(DetectRegion src/mark_recognition/detect_region.cpp)
target_link_libraries(DetectRegion ${catkin_LIBRARIES})

# executables
add_executable(odom_gazebo src/simulation/odom.cpp)
target_link_libraries(odom_gazebo ${catkin_LIBRARIES})

add_executable(tb_tf_broadcaster src/simulation/turtlebot_tf_broadcaster.cpp)
target_link_libraries(tb_tf_broadcaster ${catkin_LIBRARIES})

add_executable(fake_amcl src/simulation/fake_localization.cpp)
target_link_libraries(fake_amcl ${catkin_LIBRARIES})

add_executable(sim_coordinator src/simulation/sim_coordinator.cpp)
target_link_libraries(sim_coordinator ${catkin_LIBRARIES})

add_executable(markRecognition src/mark_recognition/mark_recognition.cpp)
target_link_libraries(markRecognition DetectRegion ${catkin_LIBRARIES})
add_dependencies(markRecognition tb_waiter_gencpp)

add_executable(turtlebot_services src/turtlebot/turtlebot_services.cpp)
target_link_libraries(turtlebot_services KobukiBase ${catkin_LIBRARIES})
add_dependencies(turtlebot_services tb_waiter_gencpp)

add_executable(turtlebotActions src/turtlebot/turtlebot_actions.cpp)
target_link_libraries(turtlebotActions KobukiBase ${catkin_LIBRARIES})
add_dependencies(turtlebotActions tb_waiter_gencpp)

add_executable(ServiceAgent src/agent_frame/contract_net/service_agent/service_agent.cpp)
target_link_libraries(ServiceAgent KobukiCommand ${catkin_LIBRARIES})
add_dependencies(ServiceAgent tb_waiter_gencpp)

add_executable(AssignerAgent src/agent_frame/contract_net/assigner_agent/assigner_agent.cpp)
target_link_libraries(AssignerAgent ${catkin_LIBRARIES})
add_dependencies(AssignerAgent tb_waiter_gencpp)

add_executable(waiterAgentDra src/agent_frame/dynamic_role/robot_agent/waiter_agent_dra.cpp)
target_link_libraries(waiterAgentDra KobukiCommand ${catkin_LIBRARIES})
add_dependencies(waiterAgentDra tb_waiter_gencpp)

add_executable(AssignerAgentDra src/agent_frame/dynamic_role/assigner_agent/assigner_agent_dra.cpp)
target_link_libraries(AssignerAgentDra KobukiCommand ${catkin_LIBRARIES})
add_dependencies(AssignerAgentDra tb_waiter_gencpp)

