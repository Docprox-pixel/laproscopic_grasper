cmake_minimum_required(VERSION 3.8)
project(laproscopic_grasper)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch config worlds scripts meshes models scripts/sofa
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/force_feedback_node.py
    scripts/grasper_controller.py
    scripts/tissue_interaction.py
    scripts/force_visualizer.py
    scripts/robot_controller_gui.py
    scripts/grasper_command_node.py
    scripts/laproscopic_gesture_control.py
    scripts/tissue_perception_node.py
    scripts/trocar_gesture_control.py
    scripts/sofa_sim_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
