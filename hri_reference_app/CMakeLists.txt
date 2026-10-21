cmake_minimum_required(VERSION 3.16)
project(hri_reference_app)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/hri_app.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hri_app
)

install(
  PROGRAMS scripts/speak_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME speak_demo
)

install(
  PROGRAMS scripts/people_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME people_demo
)

install(
  PROGRAMS scripts/move_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME move_demo
)

install(
  FILES
    scripts/capability_lifecycle.py
    scripts/speak_demo.py
    scripts/people_demo.py
    scripts/move_demo.py
    scripts/velocity_mode.py
    scripts/follow_mode.py
    scripts/follow_session.py
    scripts/follow_controller.py
    scripts/follow_motion.py
    scripts/follow_camera.py
    scripts/follow_speech.py
    scripts/camera_geometry.py
    scripts/transcribe_mode.py
    scripts/graceful_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
