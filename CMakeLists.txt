cmake_minimum_required(VERSION 3.8)
project(match_mur_gui)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(match_mur_gui)

install(PROGRAMS
  scripts/align_arm_to_plane.py
  scripts/ensure_ur_ready.py
  scripts/general_mur_gui.py
  scripts/move_arm_to_named_pose.py
  scripts/ur_dashboard_safety_check.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_alignment test/test_alignment.py)
endif()

ament_package()
