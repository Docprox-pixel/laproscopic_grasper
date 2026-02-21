cmake_minimum_required(VERSION 3.8)
project(laproscopic_grasper)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch config worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/force_feedback_node.py
    scripts/grasper_controller.py
    scripts/tissue_interaction.py
    scripts/robot_controller_gui.py  
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
