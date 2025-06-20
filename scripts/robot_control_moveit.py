#!/usr/bin/env python3

import numpy as np
import time
import transforms3d as t3d
import triad_openvr as vr
from vr_test import get_transforms

import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class XArmMoveItControl:
    def __init__(self, move_group_name="xarm_experiment"):
        # Initialize ROS node
        rospy.init_node('xarm_vr_control', anonymous=True)
        
        # Initialize MoveIt
        moveit_commander.roscpp_initialize([])
        
        # Initialize robot and planning scene
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Initialize move group
        self.move_group = moveit_commander.MoveGroupCommander(move_group_name)
        
        # Set planning parameters
        self.move_group.set_planning_time(2.0)
        self.move_group.set_num_planning_attempts(5)
        self.move_group.set_max_velocity_scaling_factor(0.3)
        self.move_group.set_max_acceleration_scaling_factor(0.3)
        
        print(f"Planning frame: {self.move_group.get_planning_frame()}")
        print(f"End effector link: {self.move_group.get_end_effector_link()}")
        print(f"Joint names: {self.move_group.get_joints()}")
        
        # Get initial TCP state
        self.get_initial_tcp_state()
        
    def get_initial_tcp_state(self):
        """Get the current TCP pose from MoveIt"""
        current_pose = self.move_group.get_current_pose().pose
        
        self.initial_tcp_position = np.array([
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z
        ])
        
        # Convert quaternion to rotation matrix
        quat = [
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w
        ]
        
        # Get Euler angles from quaternion
        euler_angles = euler_from_quaternion(quat)
        self.initial_tcp_orientation = np.array(euler_angles)  # [roll, pitch, yaw]
        
        # Get rotation matrix for coordinate transformations
        self.initial_tcp_rotation_matrix = t3d.euler.euler2mat(
            euler_angles[0], euler_angles[1], euler_angles[2], 'rxyz'
        )
        
        print(f"Initial TCP position: {self.initial_tcp_position}")
        print(f"Initial TCP orientation (RPY): {self.initial_tcp_orientation * 180/np.pi} degrees")
        print(f"Initial TCP Z-axis direction: {self.initial_tcp_rotation_matrix[:, 2]}")
        
    def set_tcp_pose(self, position, orientation_rpy, execute=True):
        """Set TCP pose using MoveIt planning"""
        # Create pose message
        pose_goal = Pose()
        pose_goal.position.x = position[0]
        pose_goal.position.y = position[1]
        pose_goal.position.z = position[2]
        
        # Convert RPY to quaternion
        quat = quaternion_from_euler(orientation_rpy[0], orientation_rpy[1], orientation_rpy[2])
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]
        pose_goal.orientation.w = quat[3]
        
        # Set pose target
        self.move_group.set_pose_target(pose_goal)
        
        if execute:
            # Plan and execute
            success = self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            return success
        else:
            # Just plan, don't execute
            plan = self.move_group.plan()
            self.move_group.clear_pose_targets()
            return plan[0]  # Return success status
            
    def get_current_tcp_pose(self):
        """Get current TCP pose"""
        current_pose = self.move_group.get_current_pose().pose
        
        position = np.array([
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z
        ])
        
        quat = [
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w
        ]
        
        orientation_rpy = np.array(euler_from_quaternion(quat))
        
        return position, orientation_rpy
        
    def close(self):
        """Clean shutdown"""
        moveit_commander.roscpp_shutdown()


class VRMoveItTeleop:
    def __init__(self):
        # Initialize VR system
        print("Initializing VR system...")
        self.vr_system = vr.triad_openvr()
        self.controller = self.vr_system.devices["controller_1"]
        print("VR system initialized")
        
        # Initialize MoveIt control
        print("Initializing MoveIt...")
        self.robot_control = XArmMoveItControl()
        print("MoveIt initialized")
        
        # VR-Robot coordinate mapping variables
        self.vr_to_robot_frame = None
        self.initial_controller_pose = None
        self.initial_tcp_pose = None
        self.previous_controller_rotation = None
        self.previous_controller_position = None
        
        # Control parameters (matching original)
        self.position_scale = 0.8  # Base scaling from original
        self.max_position_change_per_frame = 0.01  # 1cm max per frame
        self.max_rotation_per_frame = 0.1  # ~5.7 degrees max per frame
        
        # Smoothing parameters
        self.use_ema_smoothing = True
        self.ema_alpha = 0.7  # EMA smoothing factor
        self.smoothed_position = None
        self.smoothed_rotation = None
        
        self.initialize_coordinate_mapping()
        
    def initialize_coordinate_mapping(self):
        """Initialize coordinate mapping between VR and robot"""
        print("Initializing coordinate mapping...")
        
        # Flush controller data
        self.flush_controller_data()
        
        # Get VR coordinate transformation (if needed)
        # Since we assume VR world frame = base_link, we can use identity
        self.vr_to_robot_frame = np.eye(4)
        
        # Get initial controller pose
        controller_matrix = self.controller.get_pose_matrix()
        while controller_matrix is None:
            controller_matrix = self.controller.get_pose_matrix()
            time.sleep(0.01)
            
        self.initial_controller_pose = controller_matrix.copy()
        self.previous_controller_rotation = controller_matrix[:3, :3].copy()
        
        # Get initial TCP pose
        tcp_pos, tcp_rpy = self.robot_control.get_current_tcp_pose()
        self.initial_tcp_pose = {
            'position': tcp_pos.copy(),
            'orientation': tcp_rpy.copy()
        }
        
        print("Coordinate mapping initialized")
        print(f"Initial controller position: {self.initial_controller_pose[:3, 3]}")
        print(f"Initial TCP position: {tcp_pos}")
        
    def flush_controller_data(self, flush_count=50):
        """Flush old controller data"""
        for _ in range(flush_count):
            pose_matrix = self.controller.get_pose_matrix()
            if pose_matrix is None:
                continue
            time.sleep(0.001)
            
    def get_controller_pose_change(self):
        """Get controller pose change since last frame (frame-to-frame tracking)"""
        controller_matrix = self.controller.get_pose_matrix()
        if controller_matrix is None:
            return None, None
            
        current_position = controller_matrix[:3, 3]
        current_rotation = controller_matrix[:3, :3]
        
        # Frame-to-frame position tracking (like original)
        if self.previous_controller_position is not None:
            position_change = current_position - self.previous_controller_position
        else:
            position_change = np.array([0, 0, 0])  # No change on first frame
        
        # Frame-to-frame rotation tracking 
        if self.previous_controller_rotation is not None:
            # Calculate relative rotation since last frame
            relative_rotation = self.previous_controller_rotation.T @ current_rotation
            
            # Extract Y-axis rotation (around controller's local Y-axis)
            # Using the same approach as original for controller Y-axis rotation
            y_axis_rotation = np.arctan2(relative_rotation[0, 2], relative_rotation[2, 2])
        else:
            y_axis_rotation = 0.0  # No change on first frame
            
        # Update previous states
        self.previous_controller_position = current_position.copy()
        self.previous_controller_rotation = current_rotation.copy()
        
        return position_change, y_axis_rotation
            
    def map_to_tcp_pose(self, controller_pos_change, controller_y_rotation):
        """Map controller changes to TCP pose with smooth tanh mapping (like original)"""
        # Get current TCP pose
        current_tcp_pos, current_tcp_rpy = self.robot_control.get_current_tcp_pose()
        
        # SMOOTH POSITION MAPPING (following original approach)
        # Apply base scaling first
        raw_pos_change = controller_pos_change * self.position_scale
        
        # Apply smooth tanh mapping to compress large movements while preserving small ones  
        tcp_pos_change = np.tanh(raw_pos_change / self.max_position_change_per_frame) * self.max_position_change_per_frame
        
        # Apply EMA smoothing if enabled
        if self.use_ema_smoothing:
            if self.smoothed_position is None:
                self.smoothed_position = tcp_pos_change
            else:
                self.smoothed_position = (self.ema_alpha * tcp_pos_change + 
                                        (1 - self.ema_alpha) * self.smoothed_position)
            tcp_pos_change = self.smoothed_position
        
        # Calculate target position (frame-to-frame relative movement)
        target_tcp_pos = current_tcp_pos + tcp_pos_change
        
        # SMOOTH ROTATION MAPPING (following original approach)
        # Apply smooth tanh mapping for rotation: compress large movements while preserving small ones
        raw_rotation_change = controller_y_rotation
        mapped_rotation_change = np.tanh(raw_rotation_change / self.max_rotation_per_frame) * self.max_rotation_per_frame
        
        # Apply EMA smoothing to rotation if enabled
        if self.use_ema_smoothing:
            if self.smoothed_rotation is None:
                self.smoothed_rotation = mapped_rotation_change
            else:
                self.smoothed_rotation = (self.ema_alpha * mapped_rotation_change + 
                                        (1 - self.ema_alpha) * self.smoothed_rotation)
            mapped_rotation_change = self.smoothed_rotation
        
        # Apply rotation change to TCP Z-axis (yaw) - frame-to-frame relative
        target_tcp_rpy = current_tcp_rpy.copy()
        target_tcp_rpy[2] += mapped_rotation_change  # Add to yaw
        
        # Debug output for large changes (like original)
        if np.linalg.norm(tcp_pos_change) > 0.001:  # > 1mm change
            print(f"\n[POS CHANGE] Controller: [{controller_pos_change[0]*1000:.1f}, {controller_pos_change[1]*1000:.1f}, {controller_pos_change[2]*1000:.1f}]mm -> TCP: [{tcp_pos_change[0]*1000:.1f}, {tcp_pos_change[1]*1000:.1f}, {tcp_pos_change[2]*1000:.1f}]mm", end="")
            
        if abs(mapped_rotation_change) > 0.001:  # > 0.057 degrees
            rotation_deg = mapped_rotation_change * 180 / np.pi
            print(f"\n[ROTATION] Controller Y: {controller_y_rotation*180/np.pi:.3f}° -> TCP Z: {rotation_deg:.3f}°", end="")
        
        return target_tcp_pos, target_tcp_rpy
        
    def run_teleop_loop(self):
        """Main teleoperation loop"""
        print("Starting teleoperation loop...")
        print("Use menu button to exit")
        
        loop_count = 0
        start_time = time.time()
        
        try:
            while True:
                loop_start = time.time()
                
                # Get controller inputs
                controller_inputs = self.controller.get_controller_inputs()
                
                # Exit condition
                if controller_inputs.get("menu_button", False):
                    print("Menu button pressed, exiting...")
                    break
                
                # Get controller pose changes
                pos_change, y_rotation = self.get_controller_pose_change()
                
                if pos_change is not None:
                    # Map to TCP pose
                    target_pos, target_rpy = self.map_to_tcp_pose(pos_change, y_rotation)
                    
                    # Debug output
                    if loop_count % 10 == 0:  # Print every 10th loop
                        pos_change_mm = pos_change * 1000
                        y_rot_deg = y_rotation * 180 / np.pi
                        target_rpy_deg = target_rpy * 180 / np.pi
                        
                        print(f"\n[{loop_count:04d}] Ctrl pos Δ: [{pos_change_mm[0]:.1f}, {pos_change_mm[1]:.1f}, {pos_change_mm[2]:.1f}]mm, "
                              f"Y-rot: {y_rot_deg:.2f}°")
                        print(f"        TCP target: pos=[{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]m, "
                              f"rpy=[{target_rpy_deg[0]:.1f}, {target_rpy_deg[1]:.1f}, {target_rpy_deg[2]:.1f}]°")
                    
                    # Execute motion (with planning)
                    success = self.robot_control.set_tcp_pose(target_pos, target_rpy, execute=True)
                    
                    if not success:
                        print(f"Failed to plan/execute motion at loop {loop_count}")
                        # Trigger haptic feedback on failure
                        self.controller.trigger_haptic_pulse()
                
                # Gripper control with trigger
                trigger_value = controller_inputs.get('trigger', 0)
                if trigger_value > 0.5:
                    # TODO: Add gripper control via MoveIt or direct control
                    pass
                
                loop_count += 1
                
                # Control loop frequency (10 Hz)
                loop_duration = time.time() - loop_start
                target_duration = 0.1  # 10 Hz
                if loop_duration < target_duration:
                    time.sleep(target_duration - loop_duration)
                    
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        except Exception as e:
            print(f"\nError in teleop loop: {e}")
            import traceback
            traceback.print_exc()
        finally:
            print("Cleaning up...")
            self.robot_control.close()
            
    def close(self):
        """Clean shutdown"""
        self.robot_control.close()


if __name__ == "__main__":
    try:
        teleop = VRMoveItTeleop()
        teleop.run_teleop_loop()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Done.")