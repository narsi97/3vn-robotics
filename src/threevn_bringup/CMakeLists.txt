cmake_minimum_required(VERSION 3.8)
project(threevn_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # ONLY the fast test is registered with CTest. The ROS integration
  # tests carry the `ros` marker and launch a real robot, which takes
  # ~25s each -- same reasoning as threevn_sim, and ament_add_pytest_test
  # has no way to pass a marker filter.
  #
  # `make test-ros` runs them directly.
  ament_add_pytest_test(bringup_tests test/test_package_dependencies.py
    TIMEOUT 60)
  # Also fast: pure config arithmetic, no robot, no simulator.
  ament_add_pytest_test(bringup_velocity_ceilings
    test/test_velocity_ceilings.py TIMEOUT 60)
endif()

ament_package()
