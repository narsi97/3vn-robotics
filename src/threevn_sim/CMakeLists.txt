cmake_minimum_required(VERSION 3.8)
project(threevn_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds config launch
  DESTINATION share/${PROJECT_NAME}
)

# Tell Gazebo where to find this package's worlds and models. Using an
# ament environment hook (rather than a hand-set absolute path in every
# launch file) is why worlds live inside the package share directory
# instead of a top-level simulation/ folder -- the path stays correct
# wherever the workspace is checked out.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # ONLY the fast asset checks are registered with CTest.
  #
  # test_gz_spawn.py starts a real Gazebo and takes ~two minutes, so it is
  # deliberately not listed here. ament_add_pytest_test has no way to pass
  # `-m "not slow"` (see its documented parameters), and `colcon test
  # --pytest-args` does not reach a CTest-driven pytest -- so naming the
  # file explicitly is the only honest way to keep `make test` fast.
  #
  # `make test-sim` runs test_gz_spawn.py directly.
  ament_add_pytest_test(sim_asset_tests test/test_world.py TIMEOUT 60)
  # Pure dictionary work: no ROS graph, no Gazebo, milliseconds.
  ament_add_pytest_test(sim_controller_merge test/test_controller_merge.py
    TIMEOUT 60)
endif()

ament_package()
