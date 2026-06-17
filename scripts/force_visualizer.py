#!/usr/bin/env python3
"""
Force Feedback Visualizer

Listens on /surgical_robot/grip_force and plots a rolling Force vs Time
graph in real time using Matplotlib. Useful for watching what the grasper
is actually doing during a procedure.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import collections
import time
import signal
import sys

class ForceVisualizer(Node):
    def __init__(self):
        super().__init__('force_visualizer')
        
        self.get_logger().info("Force Visualizer - INITIALIZING")
        
        self.subscription = self.create_subscription(
            Float32,
            '/surgical_robot/grip_force',
            self.listener_callback,
            10)
        
        self.max_samples = 500  # roughly 10 seconds worth of data at 50 Hz
        self.times = collections.deque(maxlen=self.max_samples)
        self.forces = collections.deque(maxlen=self.max_samples)
        self.start_time = time.time()
        
        # dark theme looks better for a surgical monitoring display
        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.line, = self.ax.plot([], [], color='#00FFCC', linewidth=2.5, label='Grip Force (N)')
        
        self.ax.set_title('Real-time Surgical Force Feedback', fontsize=18, fontweight='bold', color='#FFFFFF')
        self.ax.set_xlabel('Time (seconds)', fontsize=12, color='#CCCCCC')
        self.ax.set_ylabel('Force (Newtons)', fontsize=12, color='#CCCCCC')
        
        self.ax.grid(True, linestyle='--', alpha=0.3, color='#444444')
        self.ax.set_ylim(-0.1, 3.0)  # sensible default; autoscales up if forces exceed this
        
        # horizontal reference lines so it's obvious when we're approaching danger
        self.ax.axhline(y=1.5, color='orange', linestyle='--', alpha=0.5, label='Warning (1.5N)')
        self.ax.axhline(y=2.0, color='red', linestyle='--', alpha=0.5, label='Critical (2.0N)')
        
        self.ax.legend(loc='upper right', frameon=True, facecolor='#222222', edgecolor='#444444')
        
        self.get_logger().info("Visualizer UI Ready. Waiting for data...")

    def listener_callback(self, msg):
        self.times.append(time.time() - self.start_time)
        self.forces.append(msg.data)

    def update_plot(self, frame):
        if len(self.times) > 1:
            self.line.set_data(list(self.times), list(self.forces))
            
            # keep the x-axis window sliding as new data comes in
            t_min = self.times[0]
            t_max = self.times[-1]
            self.ax.set_xlim(t_min, max(t_min + 5.0, t_max + 0.5))
            
            # stretch y-axis up if the force readings climb above the current ceiling
            curr_max = max(self.forces)
            if curr_max > self.ax.get_ylim()[1] - 0.2:
                self.ax.set_ylim(-0.1, curr_max + 1.0)
        return self.line,

def main(args=None):
    rclpy.init(args=args)
    node = ForceVisualizer()
    
    # FuncAnimation drives the plot refresh at 50 ms intervals
    ani = FuncAnimation(node.fig, node.update_plot, interval=50, cache_frame_data=False)
    
    # make Ctrl+C actually exit instead of hanging
    signal.signal(signal.SIGINT, lambda sig, frame: sys.exit(0))
    
    try:
        # spin rclpy and matplotlib together — can't block on either one
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            plt.pause(0.001)
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().info("Closing Force Visualizer...")
    finally:
        plt.close('all')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
