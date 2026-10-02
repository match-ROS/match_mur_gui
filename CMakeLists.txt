cmake_minimum_required(VERSION 3.8)
project(match_mur_gui)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(match_mur_gui)

install(PROGRAMS
  scripts/align_arm_to_plane.py
  scripts/set_cartesian_controller.py
  scripts/ensure_ur_ready.py
  scripts/general_mur_gui.py
  scripts/move_arm_to_named_pose.py
  scripts/ur_dashboard_safety_check.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_cartesian_startup test/test_cartesian_startup.py)
  ament_add_pytest_test(test_alignment test/test_alignment.py)
  ament_add_pytest_test(test_desktop_installer test/test_desktop_installer.py)
  ament_add_pytest_test(test_app_icon test/test_app_icon.py)
  ament_add_pytest_test(test_gui_layout test/test_gui_layout.py)
endif()

ament_package()
