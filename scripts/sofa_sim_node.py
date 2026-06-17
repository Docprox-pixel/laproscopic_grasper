#!/usr/bin/env python3
import Sofa
import Sofa.Core
import Sofa.Simulation
import SofaRuntime
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading
import time

import sys
import os
from ament_index_python.packages import get_package_share_directory

# try to import from the installed share path first; fall back to a relative import
# if the package isn't installed yet (useful when running directly from source)
try:
    pkg_share = get_package_share_directory('laproscopic_grasper')
    sofa_path = os.path.join(pkg_share, 'scripts', 'sofa')
    sys.path.append(sofa_path)
    from soft_tissue_scene import createScene
except Exception as e:
    print(f"Warning: Could not add SOFA path: {e}")
    from sofa.soft_tissue_scene import createScene

class SofaSimulationNode(Node):
    def __init__(self):
        super().__init__('sofa_simulation_node')
        self.get_logger().info("======================================================")
        self.get_logger().info("  SOFA Soft Tissue Simulation Node - ONLINE")
        self.get_logger().info("======================================================")

        # publish deformation metrics — currently using COM position or max stress
        self.data_pub = self.create_publisher(Float32MultiArray, '/sofa/tissue_status', 10)
        
        # spin up the SOFA scene
        self.root = Sofa.Core.Node("root")
        createScene(self.root)
        Sofa.Simulation.init(self.root)
        
        # run SOFA in its own thread so it doesn't block the ROS spin loop
        self.sim_thread = threading.Thread(target=self._run_sofa)
        self.sim_thread.daemon = True
        self.sim_thread.start()

    def _run_sofa(self):
        """Infinite loop for SOFA simulation steps."""
        dt = self.root.dt.value if hasattr(self.root, 'dt') else 0.01
        self.get_logger().info(f"SOFA Simulation Loop started (dt={dt}s)")
        
        while rclpy.ok():
            try:
                Sofa.Simulation.animate(self.root, dt)
                self._publish_status()
                # throttle to real-time simulation speed
                time.sleep(dt)
            except Exception as e:
                self.get_logger().error(f"SOFA Sim Error: {e}")
                break

    def _publish_status(self):
        """Extract physics data from SOFA and publish to ROS 2."""
        # grab first liver DOF point as a simple deformation indicator
        if hasattr(self.root, 'Liver') and hasattr(self.root.Liver, 'dofs'):
            pos = self.root.Liver.dofs.position.value[0]
            msg = Float32MultiArray()
            msg.data = [float(p) for p in pos]
            self.data_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SofaSimulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("SOFA Node shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
