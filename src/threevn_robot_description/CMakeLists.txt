cmake_minimum_required(VERSION 3.8)
project(threevn_robot_description)

find_package(ament_cmake REQUIRED)

# NOTE: we deliberately do NOT use xacro_add_files() to pre-expand the
# description at build time. It does not work with xacro.load_yaml()
# (ros/xacro#298), and pre-expanding would also create a second artifact
# that can drift from the YAML. Expansion happens at launch and test time
# only, so there is exactly one source of truth.
install(
  DIRECTORY config urdf rviz launch meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # One pytest invocation for the whole directory so conftest.py fixtures
  # (which memoise xacro expansion) are shared across files. Running each
  # file separately would re-expand every time and blow the 60s budget.
  ament_add_pytest_test(description_tests test
    TIMEOUT 120
  )
endif()

ament_package()
