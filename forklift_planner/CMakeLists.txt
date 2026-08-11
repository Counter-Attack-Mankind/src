cmake_minimum_required(VERSION 3.5)
project(forklift_planner)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-O2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  visualization_msgs
  geometry_msgs
  std_msgs
  forklift_map
  sandbox_msgs   # 实车桥:AprilObject(位姿入) / Trajectory(轨迹出)
)

catkin_package(
  CATKIN_DEPENDS roscpp roslib visualization_msgs geometry_msgs std_msgs forklift_map sandbox_msgs
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
  src/multi_vehicle/cluster_admission_counterfactual_simulator.cpp
  src/multi_vehicle/dynamic_cluster_admission_shadow.cpp
  src/multi_vehicle/footprint.cpp
  src/multi_vehicle/future_cluster_admission_shadow.cpp
  src/multi_vehicle/future_cluster_arbitration_shadow.cpp
  src/multi_vehicle/future_conflict_cluster_shadow.cpp
  src/multi_vehicle/future_conflict_zone_shadow.cpp
  src/multi_vehicle/future_mission_trajectory.cpp
  src/multi_vehicle/prediction_shadow_comparator.cpp
  src/multi_vehicle/timed_conflict_shadow_checker.cpp
  src/multi_vehicle/marker_publisher.cpp
  src/multi_vehicle/multi_vehicle_config.cpp
  src/multi_vehicle/path_track.cpp
  src/multi_vehicle/rule_engine.cpp
  src/multi_vehicle/task_allocator.cpp
  src/multi_vehicle/traffic_resource_map.cpp
  src/multi_vehicle_patrol_node.cpp
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

if(CATKIN_ENABLE_TESTING)
  add_executable(future_a1_policy_test
    test/future_a1_policy_test.cpp
  )
  add_test(NAME future_a1_policy_test COMMAND future_a1_policy_test)

  add_executable(future_mission_trajectory_test
    src/multi_vehicle/footprint.cpp
    src/multi_vehicle/future_mission_trajectory.cpp
    src/multi_vehicle/path_track.cpp
    test/future_mission_trajectory_test.cpp
  )
  add_dependencies(future_mission_trajectory_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(future_mission_trajectory_test ${catkin_LIBRARIES})
  add_test(NAME future_mission_trajectory_test
           COMMAND future_mission_trajectory_test)

  add_executable(prediction_shadow_comparator_test
    src/multi_vehicle/footprint.cpp
    src/multi_vehicle/future_mission_trajectory.cpp
    src/multi_vehicle/path_track.cpp
    src/multi_vehicle/prediction_shadow_comparator.cpp
    test/prediction_shadow_comparator_test.cpp
  )
  add_dependencies(prediction_shadow_comparator_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(prediction_shadow_comparator_test ${catkin_LIBRARIES})
  add_test(NAME prediction_shadow_comparator_test
           COMMAND prediction_shadow_comparator_test)

  add_executable(timed_conflict_shadow_checker_test
    src/multi_vehicle/footprint.cpp
    src/multi_vehicle/future_conflict_zone_shadow.cpp
    src/multi_vehicle/future_mission_trajectory.cpp
    src/multi_vehicle/path_track.cpp
    src/multi_vehicle/prediction_shadow_comparator.cpp
    src/multi_vehicle/timed_conflict_shadow_checker.cpp
    test/timed_conflict_shadow_checker_test.cpp
  )
  add_dependencies(timed_conflict_shadow_checker_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(timed_conflict_shadow_checker_test ${catkin_LIBRARIES})
  add_test(NAME timed_conflict_shadow_checker_test
           COMMAND timed_conflict_shadow_checker_test)

  add_executable(future_conflict_zone_shadow_test
    src/multi_vehicle/footprint.cpp
    src/multi_vehicle/future_conflict_zone_shadow.cpp
    src/multi_vehicle/future_mission_trajectory.cpp
    src/multi_vehicle/path_track.cpp
    test/future_conflict_zone_shadow_test.cpp
  )
  add_dependencies(future_conflict_zone_shadow_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(future_conflict_zone_shadow_test ${catkin_LIBRARIES})
  add_test(NAME future_conflict_zone_shadow_test
           COMMAND future_conflict_zone_shadow_test)

  add_executable(future_conflict_cluster_shadow_test
    src/multi_vehicle/future_conflict_cluster_shadow.cpp
    test/future_conflict_cluster_shadow_test.cpp
  )
  add_dependencies(future_conflict_cluster_shadow_test
                   ${catkin_EXPORTED_TARGETS})
  target_link_libraries(future_conflict_cluster_shadow_test
                        ${catkin_LIBRARIES})
  add_test(NAME future_conflict_cluster_shadow_test
           COMMAND future_conflict_cluster_shadow_test)

  add_executable(future_cluster_arbitration_shadow_test
    src/multi_vehicle/future_cluster_arbitration_shadow.cpp
    test/future_cluster_arbitration_shadow_test.cpp
  )
  add_dependencies(future_cluster_arbitration_shadow_test
                   ${catkin_EXPORTED_TARGETS})
  target_link_libraries(future_cluster_arbitration_shadow_test
                        ${catkin_LIBRARIES})
  add_test(NAME future_cluster_arbitration_shadow_test
           COMMAND future_cluster_arbitration_shadow_test)

  add_executable(future_cluster_admission_shadow_test
    src/multi_vehicle/future_cluster_admission_shadow.cpp
    src/multi_vehicle/future_cluster_arbitration_shadow.cpp
    test/future_cluster_admission_shadow_test.cpp
  )
  add_dependencies(future_cluster_admission_shadow_test
                   ${catkin_EXPORTED_TARGETS})
  target_link_libraries(future_cluster_admission_shadow_test
                        ${catkin_LIBRARIES})
  add_test(NAME future_cluster_admission_shadow_test
           COMMAND future_cluster_admission_shadow_test)

  add_executable(cluster_admission_counterfactual_simulator_test
    src/multi_vehicle/cluster_admission_counterfactual_simulator.cpp
    src/multi_vehicle/dynamic_cluster_admission_shadow.cpp
    src/multi_vehicle/path_track.cpp
    test/cluster_admission_counterfactual_simulator_test.cpp
  )
  add_dependencies(cluster_admission_counterfactual_simulator_test
                   ${catkin_EXPORTED_TARGETS})
  target_link_libraries(cluster_admission_counterfactual_simulator_test
                        ${catkin_LIBRARIES})
  add_test(NAME cluster_admission_counterfactual_simulator_test
           COMMAND cluster_admission_counterfactual_simulator_test)

  add_executable(dynamic_cluster_admission_shadow_test
    src/multi_vehicle/dynamic_cluster_admission_shadow.cpp
    src/multi_vehicle/path_track.cpp
    test/dynamic_cluster_admission_shadow_test.cpp
  )
  add_dependencies(dynamic_cluster_admission_shadow_test
                   ${catkin_EXPORTED_TARGETS})
  target_link_libraries(dynamic_cluster_admission_shadow_test
                        ${catkin_LIBRARIES})
  add_test(NAME dynamic_cluster_admission_shadow_test
           COMMAND dynamic_cluster_admission_shadow_test)
endif()

install(TARGETS planner_node two_vehicle_sim_node multi_vehicle_patrol_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
