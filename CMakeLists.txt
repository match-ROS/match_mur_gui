cmake_minimum_required(VERSION 3.8)
project(match_mur_gui)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(match_mur_gui)

install(PROGRAMS
  scripts/ensure_ur_ready.py
  scripts/general_mur_gui.py
  scripts/move_arm_to_named_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
