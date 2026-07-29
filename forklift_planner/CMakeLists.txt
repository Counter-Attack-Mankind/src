cmake_minimum_required(VERSION 3.5)
project(forklift_planner)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-O2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  geometry_msgs
  std_msgs
  forklift_map
  sandbox_msgs   # 实车桥:AprilObject(位姿入) / Trajectory(轨迹出)
)

catkin_package(
  CATKIN_DEPENDS roscpp visualization_msgs geometry_msgs std_msgs forklift_map sandbox_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(PATH_GENERATOR_SOURCES
  src/path_generator.cpp
  src/path_generator_internal.cpp
  src/path_generator_route.cpp
  src/path_generator_routes/path_generator_route_dispatch.cpp
  src/path_generator_routes/a1_to_b/path_generator_route.cpp
  src/path_generator_routes/b_to_a1/path_generator_route.cpp
  src/path_generator_routes/a2_to_b/path_generator_route.cpp
  src/path_generator_routes/b_to_a2/path_generator_route.cpp
)

add_executable(planner_node
  ${PATH_GENERATOR_SOURCES}
  src/planner_node.cpp
)
add_dependencies(planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(planner_node
  ${catkin_LIBRARIES}
)

add_executable(two_vehicle_sim_node
  ${PATH_GENERATOR_SOURCES}
  src/two_vehicle_sim_node.cpp
)
add_dependencies(two_vehicle_sim_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(two_vehicle_sim_node
  ${catkin_LIBRARIES}
)

add_executable(multi_vehicle_patrol_node
  ${PATH_GENERATOR_SOURCES}
  src/multi_vehicle/footprint.cpp
  src/multi_vehicle/marker_publisher.cpp
  src/multi_vehicle/multi_vehicle_config.cpp
  src/multi_vehicle/path_track.cpp
  src/multi_vehicle/rule_engine.cpp
  src/multi_vehicle/task_allocator.cpp
  src/multi_vehicle/traffic_resource_map.cpp
  src/multi_vehicle_patrol_node.cpp
)
# This target writes concise real state lines directly to stdout and keeps
# detailed diagnostics in its coordination log. Compile out legacy ROS
# DEBUG/INFO/WARN/ERROR traffic (including rollout diagnostics); FATAL remains.
target_compile_definitions(multi_vehicle_patrol_node PRIVATE
  ROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_FATAL
)
add_dependencies(multi_vehicle_patrol_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_vehicle_patrol_node
  ${catkin_LIBRARIES}
)

# 注:实车桥已并入 multi_vehicle_patrol_node(real_mode 开关,一份代码两个模式),
# 独立的 multi_vehicle_realbridge_node 已退役(避免重复实现走样)。

# [DIAG] standalone path-generator probe (no ROS runtime)
add_executable(path_debug
  ${PATH_GENERATOR_SOURCES}
  src/multi_vehicle/footprint.cpp
  src/path_debug.cpp
)
add_dependencies(path_debug ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_debug ${catkin_LIBRARIES})

install(TARGETS planner_node two_vehicle_sim_node multi_vehicle_patrol_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
