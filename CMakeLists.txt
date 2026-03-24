cmake_minimum_required(VERSION 3.8)
project(laproscopic_grasper)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch config worlds scripts meshes models
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/force_feedback_node.py
    scripts/grasper_controller.py
    scripts/tissue_interaction.py
    scripts/force_visualizer.py
    scripts/robot_controller_gui.py   # FIX #9: was missing — GUI script not installed as executable
    scripts/grasper_command_node.py   # Gap 2: /grasper/command bridge
    scripts/hand_gesture_controller.py
    scripts/tissue_perception_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
