#!/usr/bin/env python3
"""
Surgical Hand Gesture Controller (V3.2 - Stable IK)
===================================================
Tracks hand gestures using MediaPipe Tasks API and maps to 7-DOF IIWA.
Uses: /home/doc/hand_landmarker.task
"""

import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Float32
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import threading
import time
from scipy.optimize import minimize
import os

MODEL_PATH = "/home/doc/hand_landmarker.task"

class HandGestureController(Node):
    def __init__(self):
        super().__init__('hand_gesture_controller')

        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(f"MediaPipe model file NOT FOUND at: {MODEL_PATH}")
            return

        # MediaPipe Tasks API Setup
        base_options = python.BaseOptions(model_asset_path=MODEL_PATH)
        options = vision.HandLandmarkerOptions(
            base_options=base_options,
            num_hands=1,
            min_hand_detection_confidence=0.7,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.landmarker = vision.HandLandmarker.create_from_options(options)

        # ROS2 Setup
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.jaw_client = ActionClient(self, FollowJointTrajectory, '/grasper_controller/follow_joint_trajectory')
        
        # Joint Names
        self.arm_joints_names = ["iiwa14_joint1", "iiwa14_joint2", "iiwa14_joint3", "iiwa14_joint4", "iiwa14_joint5", "iiwa14_joint6", "iiwa14_joint7"]
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.create_subscription(PoseStamped, '/surgical_robot/target_tissue_pose', self._target_cb, 10)
        self.create_subscription(Float32, '/surgical_robot/grip_force', self._force_cb, 10)

        # State
        self.current_mode = "MANUAL"
        self.target_tissue_pose = None
        self.grip_force = 0.0
        self.last_pinch = False
        self.current_joints = np.array([0.0, 0.4, 0.0, -1.0, 0.0, 1.2, 0.0]) # A good "Ready" pose
        self.auto_step = 0
        
        # Link lengths for 7-DOF IK (IIWA14 approx)
        # L[6] is the tool offset (Shaft + Wrist + Jaws approx 0.28m)
        self.L = [0.36, 0.0, 0.42, 0.0, 0.4, 0.0, 0.28] 

        # Thread for Vision
        self.cv_thread = threading.Thread(target=self._run_vision)
        self.cv_thread.daemon = True
        self.cv_thread.start()

        self.get_logger().info("Hybrid Gesture Controller V3.2 - ONLINE")

    def _joint_cb(self, msg: JointState):
        if not msg.name: return
        tmp = list(self.current_joints)
        matched = False
        for i, name in enumerate(msg.name):
            if name in self.arm_joints_names:
                idx = self.arm_joints_names.index(name)
                tmp[idx] = msg.position[i]
                matched = True
        if matched:
            self.current_joints = np.array(tmp)

    def _target_cb(self, msg: PoseStamped):
        self.target_tissue_pose = msg.pose.position

    def _force_cb(self, msg: Float32):
        self.grip_force = msg.data

    def forward_kinematics(self, q):
        # Full 7-DOF FK (Simplified chain)
        # T0->1 (z-rot) -> T1->2 (y-rot) -> T2->3 (z-rot) -> T3->4 (y-rot) -> ...
        # For simplicity in reaching a 3D point, we use a slightly more complex model than planar.
        # But for teleop, we want the tip to follow the hand.
        
        # Position of J2 center
        p2 = np.array([0, 0, self.L[0]])
        
        # Direction from J2 to J4
        s1, c1 = np.sin(q[0]), np.cos(q[0])
        s2, c2 = np.sin(q[1]), np.cos(q[1])
        dir24 = np.array([c1*s2, s1*s2, c2])
        p4 = p2 + dir24 * self.L[2]
        
        # Direction from J4 to J6
        # Blend q1, q2, q3, q4
        # Simplified: q3=0 (planar)
        s4, c4 = np.sin(q[1]+q[3]), np.cos(q[1]+q[3])
        dir46 = np.array([c1*s4, s1*s4, c4])
        p6 = p4 + dir46 * self.L[4]
        
        # Direction from J6 to Tip
        s6, c6 = np.sin(q[1]+q[3]+q[5]), np.cos(q[1]+q[3]+q[5])
        dir6t = np.array([c1*s6, s1*s6, c6])
        ptip = p6 + dir6t * self.L[6]
        
        return ptip

    def ik_solve(self, target_pos, q_guess):
        # Minimize distance + joint movement + stay away from limits
        def objective(q):
            pos = self.forward_kinematics(q)
            dist_sq = np.linalg.norm(pos - target_pos)**2
            delta_q = np.linalg.norm(q - q_guess)**2
            return dist_sq + 0.1 * delta_q
        
        bounds = [(-3.0, 3.0)] * 7
        res = minimize(objective, q_guess, method='SLSQP', bounds=bounds, options={'maxiter': 15, 'ftol': 1e-3})
        return res.x

    def calculate_finger_extension(self, landmarks):
        """
        Calculates extension (0.0=curled, 1.0=straight) for each finger.
        [Thumb, Index, Middle, Ring, Pinky]
        """
        extensions = []
        wrist = landmarks[0]
        # Reference scale: Wrist to Middle MCP (landmark 9)
        ref_scale = np.linalg.norm([wrist.x - landmarks[9].x, wrist.y - landmarks[9].y])
        if ref_scale < 0.01: return [0.0]*5

        # Tips: 4, 8, 12, 16, 20
        # Bases for extension: 2, 5, 9, 13, 17
        finger_indices = [(4, 2), (8, 5), (12, 9), (16, 13), (20, 17)]
        
        for tip_idx, base_idx in finger_indices:
            tip = landmarks[tip_idx]
            base = landmarks[base_idx]
            dist = np.linalg.norm([tip.x - base.x, tip.y - base.y])
            # Normalize and clamp. 
            # For non-thumb, extension is roughly 0.1 to 1.5 times the ref_scale
            # For thumb, it depends on hand orientation, but we use a simpler heuristic.
            ext = (dist / ref_scale)
            if tip_idx == 4: # Thumb
                ext = (ext - 0.4) / 0.8
            else:
                ext = (ext - 0.3) / 1.0
            extensions.append(np.clip(ext, 0.0, 1.0))
        
        return extensions

    def _run_vision(self):
        # Try multiple camera indices (0, 1, 2) for robustness
        cap = None
        for idx in [0, 1, 2, 4]:
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                self.get_logger().info(f"Using camera at index {idx}")
                break
            cap.release()
            cap = None
        
        if cap is None:
            self.get_logger().error("COULD NOT OPEN ANY CAMERA. Check if another app is using it.")
            return

        while rclpy.ok():
            ret, img = cap.read()
            if not ret: 
                self.get_logger().warn("Failed to capture frame. Retrying...")
                time.sleep(0.1)
                continue
            
            img = cv2.flip(img, 1)
            rgb_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)
            detect_res = self.landmarker.detect(mp_image)

            if detect_res.hand_landmarks:
                for hand_lms in detect_res.hand_landmarks:
                    wrist = hand_lms[0]
                    thumb = hand_lms[4]
                    index = hand_lms[8]

                    # Dist for pinch
                    pinch = np.linalg.norm([thumb.x - index.x, thumb.y - index.y]) < 0.06
                    if pinch != self.last_pinch:
                        self._send_jaw(0.0 if pinch else 0.8)
                        self.last_pinch = pinch

                    # Map Wrist (X: 0..1, Y: 0..1, Z: 0..1)
                    # WORLD COORDS (relative to base at 0.5, 0.6, 0.9):
                    # X: 0.3..0.7, Y: 0.1..0.5, Z: 1.0..1.3
                    tx, ty, tz = wrist.x, wrist.y, 1.0 - wrist.z
                    
                    target_world = np.array([0.5 + (tx-0.5)*0.4, 0.3 + (ty-0.5)*0.4, 1.1 + (tz-0.5)*0.3])
                    
                    self.get_logger().info(f"Target: {target_world.round(3)}", throttle_duration_sec=2.0)
                    
                    if self.current_mode == "MANUAL":
                        self.get_logger().info(f"Manual Target: {target_world}", throttle_duration_sec=1.5)
                        self._drive_to(target_world)
                    
                    elif self.current_mode == "ASSIST" and self.target_tissue_pose:
                        tissue_pos = np.array([self.target_tissue_pose.x, self.target_tissue_pose.y, self.target_tissue_pose.z])
                        dist = np.linalg.norm(target_world - tissue_pos)
                        # Hover above if close
                        target = tissue_pos + np.array([0, 0, 0.08]) if dist < 0.15 else target_world
                        self._drive_to(target)
                    
                    elif self.current_mode == "AUTO":
                         if self.target_tissue_pose:
                             self._handle_auto_sequence()

                    elif self.current_mode == "FINGER":
                        extensions = self.calculate_finger_extension(hand_lms)
                        
                        # Calculate Hand Tilt (Pitch/Roll) for J6/J7
                        # Forward vector: Wrist(0) to Middle MCP(9)
                        v_fwd = np.array([hand_lms[9].x - wrist.x, hand_lms[9].y - wrist.y])
                        v_fwd /= np.linalg.norm(v_fwd) + 1e-6
                        pitch = np.arctan2(v_fwd[1], v_fwd[0]) # approx orientation in image plane
                        
                        # Side vector: Wrist(0) to Pinky MCP(17)
                        v_side = np.array([hand_lms[17].x - wrist.x, hand_lms[17].y - wrist.y])
                        v_side /= np.linalg.norm(v_side) + 1e-6
                        roll = np.arctan2(v_side[1], v_side[0])

                        # Map 7 Joints:
                        q_new = list(self.current_joints)
                        q_new[0] = (extensions[0] - 0.5) * 5.8  # Thumb  -> J1
                        q_new[1] = (extensions[1] - 0.5) * 4.0  # Index  -> J2
                        q_new[2] = (extensions[2] - 0.5) * 5.8  # Middle -> J3
                        q_new[3] = (extensions[3] - 0.5) * 4.0  # Ring   -> J4
                        q_new[4] = (extensions[4] - 0.5) * 5.8  # Pinky  -> J5
                        q_new[5] = np.clip(pitch * 2.5, -2.0, 2.0) # Pitch  -> J6
                        q_new[6] = np.clip(roll * 2.5,  -2.9, 2.9) # Roll   -> J7
                        
                        self._send_arm(np.array(q_new))
                        
                        # Visual Feedback
                        for i, ext in enumerate(extensions):
                            cv2.rectangle(img, (10 + i*40, 150), (45 + i*40, int(150 - ext*50)), (0, 255, 0), -1)
                        # Show tilt indicators
                        cv2.line(img, (250, 100), (int(250 + v_fwd[0]*40), int(100 + v_fwd[1]*40)), (255, 255, 0), 3)

            cv2.putText(img, f"MODE: {self.current_mode}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow("Gesture Control (V3.4)", img)
            key = cv2.waitKey(1)
            if key == ord('m'): self.current_mode = "MANUAL"
            elif key == ord('a'): self.current_mode = "ASSIST"
            elif key == ord('s'): self.current_mode = "AUTO"
            elif key == ord('f'): self.current_mode = "FINGER"
            elif key == 27: break

        cap.release()
        cv2.destroyAllWindows()

    def _drive_to(self, target_pos):
        q = self.ik_solve(target_pos, self.current_joints)
        self._send_arm(q)

    def _handle_auto_sequence(self):
        if not self.target_tissue_pose: return
        t = np.array([self.target_tissue_pose.x, self.target_tissue_pose.y, self.target_tissue_pose.z])
        
        if self.auto_step == 0: # Hover
             self._drive_to(t + [0, 0, 0.12])
             if np.linalg.norm(self.forward_kinematics(self.current_joints) - (t + [0, 0, 0.12])) < 0.03:
                 self.auto_step = 1
        elif self.auto_step == 1: # Open Jaws
             self._send_jaw(0.8)
             time.sleep(1.5)
             self.auto_step = 2
        elif self.auto_step == 2: # Descend
             self._drive_to(t + [0, 0, -0.01])
             if np.linalg.norm(self.forward_kinematics(self.current_joints) - (t + [0, 0, -0.01])) < 0.02:
                 self.auto_step = 3
        elif self.auto_step == 3: # Grasp
             self._send_jaw(0.0)
             time.sleep(2.0)
             self.auto_step = 4
        elif self.auto_step == 4: # Lift
             self._drive_to(t + [0, 0, 0.25])

    def _send_arm(self, q):
        if not self.arm_client.wait_for_server(timeout_sec=0.1): return
        traj = JointTrajectory()
        traj.joint_names = self.arm_joints_names
        pt = JointTrajectoryPoint(positions=q.tolist(), time_from_start=Duration(sec=0, nanosec=150000000))
        traj.points.append(pt)
        self.arm_client.send_goal_async(FollowJointTrajectory.Goal(trajectory=traj))

    def _send_jaw(self, val):
        if not self.jaw_client.wait_for_server(timeout_sec=0.1): return
        traj = JointTrajectory()
        # Ensure all 3 grasper joints are commanded to avoid "missing joint" errors
        traj.joint_names = ["grasper_pitch_joint", "grasper_yaw_joint", "grasper_jaw_joint"]
        # Default pitch/yaw to 0.0 for stable forward pointing
        pos = [0.0, 0.0, float(val)] 
        pt = JointTrajectoryPoint(positions=pos, time_from_start=Duration(sec=0, nanosec=300000000))
        traj.points.append(pt)
        self.jaw_client.send_goal_async(FollowJointTrajectory.Goal(trajectory=traj))

def main(args=None):
    rclpy.init(args=args)
    try:
        node = HandGestureController()
        if hasattr(node, 'landmarker'):
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
