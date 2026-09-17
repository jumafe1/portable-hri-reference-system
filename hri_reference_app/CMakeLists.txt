cmake_minimum_required(VERSION 3.16)
project(hri_reference_app)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/speak_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME speak_demo
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
