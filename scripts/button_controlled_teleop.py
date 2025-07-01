import numpy as np
import sys
import os

# Add parent directory to path to find triad_openvr
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import triad_openvr as vr
import time
import transforms3d as t3d
from calibrated_transforms import get_transforms_calibrated as get_transforms
import matplotlib.pyplot as plt

# Import the official xArm SDK
from xarm.wrapper import XArmAPI

class XArmControl:
    def __init__(self, ip, mode=0, simulated=True, tcp_z_offset=188):
        self.arm = XArmAPI(ip)
        print(f"Connecting to xArm at {ip}...")
        
        # Clear warnings and errors
        self.arm.clean_warn()
        self.arm.clean_error()
        
        # Check connection and robot state
        if not self.check_connection():
            raise Exception("Failed to connect to xArm. Please check if the robot is powered on and connected.")
        
        # Print diagnostic information
        self.print_robot_state()
        
        # Initialize the robot
        self.initialize_robot(mode)
        
        # Set TCP offset
        self.arm.set_tcp_offset([0, 0, tcp_z_offset, 0, 0, 0])
        
        self.simulated = simulated
    
    def print_robot_state(self):
        """Print detailed robot state information"""
        print("\n=== Robot State Information ===")
        code, state = self.arm.get_state()
        
        states = {
            0: 'READY',
            1: 'SUSPENDED',
            2: 'STOPPED',
            3: 'PLAYING',
            4: 'PAUSED'
        }
        
        print(f"Current State: {states.get(state, f'UNKNOWN({state})')}")
        
        # Get current position
        code, pos = self.arm.get_position()
        if code == 0:
            print(f"Current Position (mm): x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}")
            if len(pos) > 3:
                print(f"Current Orientation (rad): rx={pos[3]:.2f}, ry={pos[4]:.2f}, rz={pos[5]:.2f}")
        
        # Get current joint angles
        code, angles = self.arm.get_servo_angle(is_radian=True)
        if code == 0:
            angles_deg = [angle * 180 / 3.14159 for angle in angles]
            print(f"Current Joint Angles (deg): {[f'{a:.2f}' for a in angles_deg]}")
        
        print("===========================\n")
    
    def check_connection(self):
        """Check if robot is connected properly"""
        state = self.arm.get_state()
        if state[0] == 0:  # Connected successfully
            return True
        return False
    
    def initialize_robot(self, mode):
        """Initialize robot with proper settings"""
        print("Initializing robot...")
        
        # Check current robot state first
        print("Checking current robot state...")
        code, state = self.arm.get_state()
        print(f"Current state code: {code}, state: {state}")
        
        # Clear errors and warnings
        print("Clearing errors and warnings...")
        self.arm.clean_error()
        self.arm.clean_warn()
        time.sleep(1.0)
        
        # Check if robot needs to be powered on
        print("Enabling motion...")
        ret = self.arm.motion_enable(enable=True)
        if ret != 0:
            print(f"Motion enable failed with code: {ret}")
            if ret == 9:
                print("Emergency stop is active! Please check robot safety systems.")
                raise Exception(f"Cannot enable motion due to emergency stop")
        print("Motion enabled successfully")
        time.sleep(1.0)
        
        # Set mode carefully
        print(f"Setting mode to {mode}...")
        ret = self.arm.set_mode(mode)
        if ret != 0:
            print(f"Set mode failed with code: {ret}")
            raise Exception(f"Failed to set robot mode, error code: {ret}")
        print(f"Mode set to {mode} successfully")
        time.sleep(1.0)
            
        # Set state to ready
        print("Setting state to READY...")
        ret = self.arm.set_state(state=0)
        if ret != 0:
            print(f"Set state failed with code: {ret}")
            raise Exception(f"Failed to set robot state, error code: {ret}")
        print("Robot state set to READY successfully")
        time.sleep(1.0)
        
        # Initialize TCP orientation to standard pose
        # self.initialize_tcp_orientation()  # Commented out - no need to reset pose
        
        # Print final state
        print("Final robot state:")
        self.print_robot_state()
        
    # def initialize_tcp_orientation(self):
    #     """Initialize robot to standard TCP orientation"""
    #     print("\nInitializing TCP orientation...")
    #     
    #     # Get current position
    #     code, current_pos = self.arm.get_position(is_radian=True)
    #     if code != 0:
    #         print(f"Failed to get current position, code: {code}")
    #         return
    #     
    #     # Set target orientation: roll=3.136, pitch=0.000, yaw=0.000
    #     target_orientation = [3.136, 0.000, 3.137]  # [roll, pitch, yaw] in radians
    #     
    #     # Keep current position, set target orientation
    #     target_pose = [current_pos[0], current_pos[1], current_pos[2]] + target_orientation
    #     
    #     print(f"Moving to initialization pose:")
    #     print(f"  Position (mm): x={target_pose[0]:.1f}, y={target_pose[1]:.1f}, z={target_pose[2]:.1f}")
    #     print(f"  Orientation (rad): roll={target_pose[3]:.3f}, pitch={target_pose[4]:.3f}, yaw={target_pose[5]:.3f}")
    #     
    #     # Check if target pose is reachable using inverse kinematics
    #     print("Checking if target pose is reachable...")
    #     code, joint_angles = self.arm.get_inverse_kinematics(target_pose, input_is_radian=True, return_is_radian=True)
    #     if code != 0:
    #         print(f"❌ Target pose is not reachable! IK failed with code: {code}")
    #         return
    #     else:
    #         print("✅ Target pose is reachable")
    #     
    #     # Reset robot state completely before movement to ensure clean state
    #     print("Resetting robot state before movement...")
    #     self.arm.clean_error()
    #     self.arm.clean_warn()
    #     time.sleep(1.0)
    #     
    #     # Re-enable motion and set state
    #     self.arm.motion_enable(enable=True)
    #     time.sleep(1.0)
    #     self.arm.set_mode(0)  # Position mode
    #     time.sleep(1.0)
    #     self.arm.set_state(state=0)  # Ready state
    #     time.sleep(1.0)
    #     
    #     # Use joint control with very slow movement to avoid emergency stops
    #     print("Moving to target orientation using slow joint control...")
    #     ret = self.arm.set_servo_angle(angle=joint_angles, speed=1, wait=True, is_radian=True)
    #     if ret != 0:
    #         print(f"Failed to move to initialization pose, code: {ret}")
    #         if not self.handle_error(ret, "initialize_tcp_orientation"):
    #             print("Initialization pose failed")
    #     else:
    #         print("Successfully moved to initialization pose")
    #     
    #     time.sleep(2.0)  # Allow time for movement to complete
    #     
    #     # Verify final orientation
    #     code, final_pos = self.arm.get_position(is_radian=True)
    #     if code == 0:
    #         final_orientation = final_pos[3:6]
    #         orientation_error = [abs(final_orientation[i] - target_orientation[i]) for i in range(3)]
    #         max_error = max(orientation_error)
    #         if max_error < 0.05:  # < 2.9 degrees
    #             print(f"✅ TCP orientation initialization successful")
    #             print(f"  Final orientation (rad): roll={final_orientation[0]:.3f}, pitch={final_orientation[1]:.3f}, yaw={final_orientation[2]:.3f}")
    #         else:
    #             print(f"⚠️ TCP orientation initialization completed with error: {max_error:.3f} rad ({max_error*180/3.14159:.1f} deg)")
    #     else:
    #         print("Could not verify final orientation")
    
    def handle_error(self, ret, operation=""):
        """Handle common error codes"""
        if ret == 9:  # Emergency stop
            print(f"\nEmergency stop detected during {operation}. Attempting to recover...")
            self.arm.clean_error()
            time.sleep(0.5)
            self.arm.motion_enable(enable=True)
            time.sleep(0.5)
            self.arm.set_state(state=0)
            time.sleep(0.5)
            self.print_robot_state()  # Print state after recovery attempt
            return True
        elif ret != 0:
            print(f"Error during {operation}, code: {ret}")
            return False
        return True
        
    def set_eef_position(self, x, y, z):
        # Convert to mm for xArm API
        pos = [x*1000, y*1000, z*1000]
        # Get current pose with explicit radian specification
        code, curr_pos = self.arm.get_position(is_radian=True)
        if code != 0:
            print(f"Failed to get current position for set_eef_position, code: {code}")
            return
        # Maintain current orientation - curr_pos[3:6] are in radians
        ret = self.arm.set_position(*pos, curr_pos[3], curr_pos[4], curr_pos[5], wait=True, is_radian=True)
        if not self.handle_error(ret, "set_eef_position"):
            print(f"Position command failed with code: {ret}")
        
    def set_eef_pose(self, x, y, z, roll=None, pitch=None, yaw=None):
        # Set both position and orientation
        pos = [x*1000, y*1000, z*1000]
        if roll is not None and pitch is not None and yaw is not None:
            # Set position and orientation - assume rpy values are in radians
            ret = self.arm.set_position(*pos, roll, pitch, yaw, wait=True, is_radian=True)
        else:
            # Maintain current orientation if not specified
            code, curr_pos = self.arm.get_position(is_radian=True)
            if code != 0:
                print(f"Failed to get current position for set_eef_pose, code: {code}")
                return
            ret = self.arm.set_position(*pos, curr_pos[3], curr_pos[4], curr_pos[5], wait=True, is_radian=True)
        if not self.handle_error(ret, "set_eef_pose"):
            print(f"Pose command failed with code: {ret}")
        
    def get_eef_position(self):
        # Convert from mm to meters
        ret, pos = self.arm.get_position()
        if ret == 0:
            return np.array(pos[:3]) / 1000
        self.handle_error(ret, "get_eef_position")
        return None
        
    def set_joint_velocity(self, velocities, duration=0.1):
        ret = self.arm.vc_set_joint_velocity(velocities, is_radian=True)
        if self.handle_error(ret, "set_joint_velocity"):
            time.sleep(duration)
        
    def open_gripper(self):
        ret = self.arm.set_gripper_position(850, wait=True)
        self.handle_error(ret, "open_gripper")
        
    def close_gripper(self):
        ret = self.arm.set_gripper_position(0, wait=True)
        self.handle_error(ret, "close_gripper")
        
    def reset(self):
        try:
            ret = self.arm.reset(wait=True)
            if ret == 0:
                print("Robot reset completed")
            elif ret == 9:
                print("Robot reset completed (emergency stop during reset is normal)")
            else:
                print(f"Robot reset completed with code: {ret}")
        except Exception as e:
            print(f"Reset completed with exception: {e}")
        
    def close(self):
        self.arm.disconnect()


def flush_controller_data(flush_count=50):
    pose_matrix = controller.get_pose_matrix()
    while flush_count > 0:
        pose_matrix = controller.get_pose_matrix()
        if pose_matrix is None:
            continue
        flush_count -= 1


class ButtonControlledTeleop:
    def __init__(self, controller, xarm, control_freq=10):
        self.controller = controller
        self.xarm = xarm
        self.control_freq = control_freq
        
        # State variables
        self.is_teleoperation_active = False
        self.grab_button_pressed = False
        self.previous_grab_state = False
        
        # Initial poses - will be set when grab button is first pressed
        self.frame_1_to_2 = None
        self.init_controller_position = None
        self.init_controller_orientation = None
        self.init_eef_pos = None
        self.init_tcp_orientation = None
        
        # Trajectory logging
        self.trajectory = []
        self.gripper_close_times = []
        self.gripper_open_times = []
        self.loop_count = 0
        self.gripper_closed = False
        
    def capture_initial_poses(self):
        """Capture initial poses when grab button is first pressed"""
        print("\n=== Capturing Initial Poses ===")
        
        # Flush controller data to get fresh readings
        flush_controller_data()
        
        # Get calibrated transforms
        self.frame_1_to_2, _, _ = get_transforms()
        
        # Get controller pose
        pose_matrix = self.controller.get_pose_matrix()
        while pose_matrix is None:
            pose_matrix = self.controller.get_pose_matrix()
        
        # Store initial controller position and orientation
        self.init_controller_position = self.frame_1_to_2 @ pose_matrix[:3, 3]
        self.init_controller_orientation = pose_matrix[:3, :3]
        
        # Get current robot TCP position and orientation at trigger moment
        self.init_eef_pos = self.xarm.get_eef_position()
        code, current_tcp_pose = self.xarm.arm.get_position(is_radian=True)
        if code == 0:
            self.init_tcp_orientation = np.array(current_tcp_pose[3:6])  # [roll, pitch, yaw] in radians
            print(f"Captured actual TCP orientation: [{self.init_tcp_orientation[0]:.3f}, {self.init_tcp_orientation[1]:.3f}, {self.init_tcp_orientation[2]:.3f}] rad")
        else:
            print(f"❌ Failed to get initial TCP orientation, code: {code}")
            print("⚠️ This may cause orientation issues - please check robot connection")
            self.init_tcp_orientation = np.array([0.0, 0.0, 0.0])  # Use current pose as-is
        
        print(f"Initial controller position: {self.init_controller_position}")
        print(f"Initial TCP position: {self.init_eef_pos}")
        print("=== Initial Poses Captured ===\n")
    
    def get_relative_pose(self, orientation_only=False):
        """Calculate relative pose from initial grab position
        
        Args:
            orientation_only: If True, only return orientation changes (keep current position)
        """
        pose_matrix = self.controller.get_pose_matrix()
        if pose_matrix is None:
            return None, None
        
        # Get current controller position and orientation in robot frame
        curr_controller_pos = self.frame_1_to_2 @ pose_matrix[:3, 3]
        curr_controller_ori = pose_matrix[:3, :3]
        
        # Calculate relative orientation change
        rel_controller_rotation = self.init_controller_orientation.T @ curr_controller_ori
        
        if orientation_only:
            # Orientation-only mode: keep current TCP position, only change orientation
            target_eef_pos = self.xarm.get_eef_position()
            if target_eef_pos is None:
                # Fallback to initial position if can't read current position
                target_eef_pos = self.init_eef_pos
        else:
            # Normal mode: apply relative position changes
            controller_pos_offset = curr_controller_pos - self.init_controller_position
            target_eef_pos = self.init_eef_pos + controller_pos_offset * 0.5
        
        # Extract rotation angles from relative rotation matrix using transforms3d
        # Convert rotation matrix to Euler angles (XYZ convention: roll, pitch, yaw)
        roll, pitch, yaw = t3d.euler.mat2euler(rel_controller_rotation, axes='sxyz')
        
        # Apply relative rotations to initial TCP orientation
        target_rpy = [
            self.init_tcp_orientation[0] - yaw * 0.15,   # Roll (X-axis rotation)
            self.init_tcp_orientation[1] - roll* 0.15,  # Pitch (Y-axis rotation)
            self.init_tcp_orientation[2] + pitch * 0.15     # Yaw (Z-axis rotation)
        ]
        
        return target_eef_pos, target_rpy
    
    def check_trigger_button(self):
        """Check trigger button state and handle transitions"""
        controller_inputs = self.controller.get_controller_inputs()
        
        # Check if trigger is pressed (analog value > threshold)
        trigger_value = controller_inputs.get('trigger', 0)
        current_trigger_state = trigger_value > 0.1  # Small threshold to avoid noise
        
        # Detect button press (transition from not pressed to pressed)
        if current_trigger_state and not self.previous_grab_state:
            print("🎯 TRIGGER PRESSED - Starting teleoperation")
            self.capture_initial_poses()
            self.is_teleoperation_active = True
            self.previous_grab_state = True
            return True
        
        # Detect button release (transition from pressed to not pressed)
        elif not current_trigger_state and self.previous_grab_state:
            print("🛑 TRIGGER RELEASED - Stopping teleoperation")
            self.is_teleoperation_active = False
            self.previous_grab_state = False
            return True
        
        # Update previous state
        self.previous_grab_state = current_trigger_state
        return False
    
    def run_teleoperation_loop(self):
        """Main teleoperation loop"""
        print("🚀 Starting Button-Controlled Teleoperation")
        print("📋 Instructions:")
        print("   - Press and hold TRIGGER to start teleoperation")
        print("   - Press TRACKPAD + TRIGGER for orientation-only mode")
        print("   - Release TRIGGER to stop teleoperation")
        print("   - Use GRIP BUTTON for gripper control")
        print("   - Press MENU BUTTON to exit")
        print("   - Waiting for trigger press...\n")
        
        try:
            while True:
                loop_start_time = time.time()
                
                # Check trigger button state
                self.check_trigger_button()
                
                # Only perform teleoperation if trigger is pressed
                if self.is_teleoperation_active:
                    # Check if trackpad is pressed for orientation-only mode
                    controller_inputs = self.controller.get_controller_inputs()
                    orientation_only = controller_inputs.get('trackpad_pressed', False)
                    
                    # Get relative pose from initial grab position
                    target_eef_pos, target_rpy = self.get_relative_pose(orientation_only=orientation_only)
                    
                    if target_eef_pos is not None and target_rpy is not None:
                        # Log trajectory
                        self.trajectory.append(target_eef_pos)
                        
                        # Send pose command to robot
                        pos_mm = target_eef_pos * 1000  # Convert to mm
                        rpy_deg = np.array(target_rpy) * 180 / np.pi  # Convert to degrees for display
                        
                        # Show mode and target pose
                        mode_indicator = "🔄 ORIENT-ONLY" if orientation_only else "🎯 FULL"
                        print(f"\r[{mode_indicator}] Target: pos=[{pos_mm[0]:.0f}, {pos_mm[1]:.0f}, {pos_mm[2]:.0f}]mm, "
                              f"rpy=[{rpy_deg[0]:.0f}°, {rpy_deg[1]:.0f}°, {rpy_deg[2]:.0f}°]", end="")
                        
                        # Send command to robot
                        self.xarm.set_eef_pose(target_eef_pos[0], target_eef_pos[1], target_eef_pos[2], 
                                             target_rpy[0], target_rpy[1], target_rpy[2])
                        
                        # Handle gripper control
                        self.handle_gripper_control()
                
                # Check for exit condition
                controller_inputs = self.controller.get_controller_inputs()
                if controller_inputs.get("menu_button", False):
                    print("\n🏁 Menu button pressed - Exiting teleoperation")
                    break
                
                # Maintain control frequency
                self.loop_count += 1
                loop_duration = time.time() - loop_start_time
                if loop_duration < 1/self.control_freq:
                    time.sleep(1/self.control_freq - loop_duration)
                    
        except KeyboardInterrupt:
            print("\n⏹️ Teleoperation interrupted by user")
        except Exception as e:
            print(f"\n❌ Error in teleoperation loop: {e}")
        finally:
            self.cleanup()
    
    def handle_gripper_control(self):
        """Handle gripper control with grip button"""
        controller_inputs = self.controller.get_controller_inputs()
        grip_pressed = controller_inputs.get('grip_button', False)
        
        # Close gripper when grip button is pressed
        if grip_pressed and not self.gripper_closed:
            self.gripper_close_times.append(self.loop_count)
            self.xarm.close_gripper()
            self.gripper_closed = True
            print(f"\n🤏 Closing gripper")
        
        # Open gripper when grip button is released
        elif not grip_pressed and self.gripper_closed:
            self.gripper_open_times.append(self.loop_count)
            self.xarm.open_gripper()
            self.gripper_closed = False
            print(f"\n✋ Opening gripper")
    
    def cleanup(self):
        """Cleanup and show trajectory plot"""
        print("\n🧹 Cleaning up...")
        
        # Close robot connection without reset (keep current pose)
        print("📍 Robot will remain at current pose")
        self.xarm.close()
        
        # Show trajectory plot if we have data
        if len(self.trajectory) > 0:
            plt.figure("Button-Controlled Teleoperation Trajectory")
            trajectory_array = np.array(self.trajectory)
            plt.plot(trajectory_array[:, 0], label='x')
            plt.plot(trajectory_array[:, 1], label='y') 
            plt.plot(trajectory_array[:, 2], label='z')
            plt.vlines(self.gripper_close_times, 
                      np.min(trajectory_array), np.max(trajectory_array), 
                      'r', linestyles='dashed', label="Close gripper")
            plt.vlines(self.gripper_open_times, 
                      np.min(trajectory_array), np.max(trajectory_array), 
                      'g', linestyles='dashed', label="Open gripper")
            plt.xlabel('Time steps')
            plt.ylabel('Position (m)')
            plt.title('Button-Controlled Teleoperation Trajectory')
            plt.legend()
            plt.show()
        
        print("✅ Cleanup completed")


if __name__ == "__main__":
    # Configuration
    simulated = False
    control_freq = 10  # Hz
    
    try:
        print("🔧 Initializing VR system...")
        v = vr.triad_openvr()
        print("✅ VR system initialized successfully")
        
        print("🎮 Connecting to controller...")
        controller = v.devices["controller_1"]
        print("✅ Controller connected successfully")
        
        print("🦾 Initializing xArm...")
        xarm = XArmControl(
            ip="192.168.1.226", 
            mode=0,  # Position mode
            simulated=simulated,
            tcp_z_offset=188
        )
        print("✅ xArm initialized successfully")
        
        # Create and run button-controlled teleoperation
        teleop = ButtonControlledTeleop(controller, xarm, control_freq)
        teleop.run_teleoperation_loop()
        
    except KeyboardInterrupt:
        print("\n⏹️ Control interrupted by user")
        if 'xarm' in locals():
            print("📍 Robot will remain at current pose")
            print("✅ Teleoperation stopped safely")
    except Exception as e:
        print(f"\n❌ Error occurred: {str(e)}")
        print(f"Error type: {type(e).__name__}")
        import traceback
        print("Full traceback:")
        traceback.print_exc()
    finally:
        if 'xarm' in locals():
            print("\n🔌 Closing robot connection...")
            # xarm.reset()  # Commented out - keep robot at current pose
            xarm.close()
        print("🏁 Done.")