cmake_minimum_required(VERSION 3.5)
project(gb_manipulation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_bt_actions REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(ros2_knowledge_graph REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(dependencies
    rclcpp
    rclcpp_action
    geometry_msgs
    tf2_geometry_msgs
    plansys2_msgs
    plansys2_executor
    ament_index_cpp
    plansys2_bt_actions
    moveit_msgs
    ros2_knowledge_graph
)

include_directories(include)

add_library(plansys2_pick_bt_node SHARED src/behavior_tree_nodes/Pick.cpp)
list(APPEND plugin_libs plansys2_pick_bt_node)

add_library(plansys2_place_bt_node SHARED src/behavior_tree_nodes/Place.cpp)
list(APPEND plugin_libs plansys2_place_bt_node)

add_library(plansys2_place_simple_bt_node SHARED src/behavior_tree_nodes/PlaceSimple.cpp)
list(APPEND plugin_libs plansys2_place_simple_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

#add_executable(dummy_controller_node src/dummy_controller_node.cpp)
#ament_target_dependencies(dummy_controller_node ${dependencies})

add_executable(worldtf_to_map src/utils/worldtf_to_map.cpp)
ament_target_dependencies(worldtf_to_map ${dependencies})

install(DIRECTORY launch pddl behavior_trees_xml config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  #dummy_controller_node
  worldtf_to_map
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()