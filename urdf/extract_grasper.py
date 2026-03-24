import sys
import os

in_path = '/home/doc/surgical_robot_ws/src/laproscopic_grasper/urdf/laproscopic_grasper.urdf.xacro'
out_path = '/home/doc/Downloads/fusdintry/src/assem1_description/urdf/grasper.xacro'

with open(in_path, 'r') as f:
    lines = f.readlines()

out = []
out.append('<?xml version="1.0"?>\n')
out.append('<robot xmlns:xacro="http://www.ros.org/wiki/xacro">\n')

# lines 32 to 39: materials (1-indexed) -> 31:39
out.extend(lines[31:39])

out.append('  <xacro:macro name="laproscopic_grasper" params="parent *origin">\n')
out.append('    <joint name="grasper_attach_joint" type="continuous">\n')
out.append('      <parent link="${parent}"/>\n')
out.append('      <child link="trocar_shaft"/>\n')
out.append('      <xacro:insert_block name="origin"/>\n')
out.append('      <axis xyz="0 0 1"/>\n')
out.append('      <limit lower="-3.14159" upper="3.14159" effort="10" velocity="1.0"/>\n')
out.append('    </joint>\n')

# lines 267 to 689: grasper physically starting from trocar_shaft (1-indexed) -> 266:689
out.extend(lines[266:689])

out.append('  </xacro:macro>\n')
out.append('</robot>\n')

os.makedirs(os.path.dirname(out_path), exist_ok=True)
with open(out_path, 'w') as f:
    f.writelines(out)
