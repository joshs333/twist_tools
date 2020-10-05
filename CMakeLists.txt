cmake_minimum_required(VERSION 3.0.2)
project(twist_tools)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  std_srvs
  message_generation
)

# Generate Service
add_service_files(
  FILES
  RequestTopic.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Twist Mux Node
add_executable(twist_mux_node src/twist_mux_node.cpp)
add_dependencies(twist_mux_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(twist_mux_node ${catkin_LIBRARIES})

# Twist Stamped Mux Node
add_executable(twist_stamped_mux_node src/twist_mux_node.cpp)
add_dependencies(twist_stamped_mux_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(twist_stamped_mux_node ${catkin_LIBRARIES})

# Twist Stamper Node
add_executable(twist_stamper_node src/twist_stamper_node.cpp)
add_dependencies(twist_stamper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(twist_stamper_node ${catkin_LIBRARIES})

# Twist Unstamper Node
add_executable(twist_unstamper_node src/twist_unstamper_node.cpp)
add_dependencies(twist_unstamper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(twist_unstamper_node ${catkin_LIBRARIES})

install(TARGETS twist_mux_node twist_stamped_mux_node twist_stamper_node twist_unstamper_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
