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
  # Gazebo tests are slow and excluded from `make test` by the `slow`
  # marker; `make test-sim` runs them.
  ament_add_pytest_test(sim_tests test TIMEOUT 300)
endif()

ament_package()
