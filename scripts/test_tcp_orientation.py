#!/usr/bin/env python3

from xarm.wrapper import XArmAPI
import time

class TCPOrientationTester:
    def __init__(self, ip, tcp_z_offset=218):
        self.arm = XArmAPI(ip)
        print(f"Connecting to xArm at {ip}...")
        
        # Clear warnings and errors
        self.arm.clean_warn()
        self.arm.clean_error()
        
        # Check connection and robot state
        if not self.check_connection():
            raise Exception("Failed to connect to xArm. Please check if the robot is powered on and connected.")
        
        # Initialize the robot
        self.initialize_robot()
        
        # Set TCP offset
        self.arm.set_tcp_offset([0, 0, tcp_z_offset, 0, 0, 0])
        print(f"TCP offset set to: [0, 0, {tcp_z_offset}, 0, 0, 0]")
        
    def check_connection(self):
        """Check if robot is connected properly"""
        state = self.arm.get_state()
        if state[0] == 0:  # Connected successfully
            return True
        return False
    
    def initialize_robot(self):
        """Initialize robot with basic settings"""
        print("Initializing robot...")
        
        # Clear errors and warnings
        print("Clearing errors and warnings...")
        self.arm.clean_error()
        self.arm.clean_warn()
        time.sleep(1.0)
        
        # Enable motion
        print("Enabling motion...")
        ret = self.arm.motion_enable(enable=True)
        if ret != 0:
            print(f"Motion enable failed with code: {ret}")
            if ret == 9:
                print("Emergency stop is active! Please check robot safety systems.")
                raise Exception(f"Cannot enable motion due to emergency stop")
        print("Motion enabled successfully")
        time.sleep(1.0)
        
        # Set mode to position mode
        print("Setting mode to position mode...")
        ret = self.arm.set_mode(0)
        if ret != 0:
            print(f"Set mode failed with code: {ret}")
            raise Exception(f"Failed to set robot mode, error code: {ret}")
        print("Mode set successfully")
        time.sleep(1.0)
            
        # Set state to ready
        print("Setting state to READY...")
        ret = self.arm.set_state(state=0)
        if ret != 0:
            print(f"Set state failed with code: {ret}")
            raise Exception(f"Failed to set robot state, error code: {ret}")
        print("Robot state set to READY successfully")
        time.sleep(1.0)
    
    def print_current_pose(self, label=""):
        """Print current TCP pose"""
        code, pos = self.arm.get_position(is_radian=True)
        if code == 0:
            print(f"{label}Current TCP Pose:")
            print(f"  Position (mm): x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}")
            print(f"  Orientation (rad): roll={pos[3]:.3f}, pitch={pos[4]:.3f}, yaw={pos[5]:.3f}")
            print(f"  Orientation (deg): roll={pos[3]*180/3.14159:.1f}, pitch={pos[4]*180/3.14159:.1f}, yaw={pos[5]*180/3.14159:.1f}")
            return pos
        else:
            print(f"Failed to get position, code: {code}")
            return None
    
    def initialize_tcp_orientation(self):
        """Initialize robot to standard TCP orientation"""
        print("\n" + "="*50)
        print("TESTING TCP ORIENTATION INITIALIZATION")
        print("="*50)
        
        # Print current pose before initialization
        print("\nBEFORE initialization:")
        current_pos = self.print_current_pose()
        if current_pos is None:
            return False
        
        print("\nInitializing TCP orientation...")
        
        # Set target orientation: roll=3.136, pitch=0.000, yaw=0.000
        target_orientation = [3.136, 0.000, 0.000]  # [roll, pitch, yaw] in radians
        
        # Keep current position, set target orientation
        target_pose = [current_pos[0], current_pos[1], current_pos[2]] + target_orientation
        
        print(f"\nMoving to initialization pose:")
        print(f"  Position (mm): x={target_pose[0]:.1f}, y={target_pose[1]:.1f}, z={target_pose[2]:.1f}")
        print(f"  Orientation (rad): roll={target_pose[3]:.3f}, pitch={target_pose[4]:.3f}, yaw={target_pose[5]:.3f}")
        print(f"  Orientation (deg): roll={target_pose[3]*180/3.14159:.1f}, pitch={target_pose[4]*180/3.14159:.1f}, yaw={target_pose[5]*180/3.14159:.1f}")
        
        # Check if target pose is reachable using inverse kinematics
        print("\nChecking if target pose is reachable...")
        code, joint_angles = self.arm.get_inverse_kinematics(target_pose, input_is_radian=True, return_is_radian=True)
        if code != 0:
            print(f"❌ Target pose is not reachable! IK failed with code: {code}")
            print("Possible reasons:")
            print("  - Position outside robot workspace")
            print("  - Orientation not achievable at this position")
            print("  - Joint limits exceeded")
            return False
        else:
            print("✅ Target pose is reachable")
            joint_angles_deg = [angle * 180 / 3.14159 for angle in joint_angles]
            print(f"Required joint angles (deg): {[f'{a:.1f}' for a in joint_angles_deg]}")

        # Get current joint angles for comparison
        code, current_joints = self.arm.get_servo_angle(is_radian=True)
        if code == 0:
            current_joints_deg = [j * 180 / 3.14159 for j in current_joints]
            print(f"Current joint angles (deg): {[f'{a:.1f}' for a in current_joints_deg]}")
            print(f"Target joint angles (deg): {[f'{a:.1f}' for a in joint_angles_deg]}")
            
            # Calculate joint differences
            joint_diffs = [abs(t - c) for t, c in zip(joint_angles, current_joints)]
            joint_diffs_deg = [d * 180 / 3.14159 for d in joint_diffs]
            print(f"Joint differences (deg): {[f'{d:.1f}' for d in joint_diffs_deg]}")
            max_joint_diff = max(joint_diffs_deg)
            print(f"Maximum joint change required: {max_joint_diff:.1f} degrees")
            
            if max_joint_diff > 90:
                print("⚠️ Large joint movement required - this might cause emergency stop")

        # Reset robot state completely before movement
        print("\nResetting robot state before movement...")
        self.arm.clean_error()
        self.arm.clean_warn()
        time.sleep(1.0)
        
        # Re-enable motion and set state
        self.arm.motion_enable(enable=True)
        time.sleep(1.0)
        self.arm.set_mode(0)  # Position mode
        time.sleep(1.0)
        self.arm.set_state(state=0)  # Ready state
        time.sleep(1.0)
        
        # Try extremely slow joint movement
        print("\nTrying very slow joint movement (1 deg/s)...")
        ret = self.arm.set_servo_angle(angle=joint_angles, speed=1, wait=True, is_radian=True)
        if ret != 0:
            print(f"Failed to move to initialization pose, code: {ret}")
            
            # Print error code meanings
            error_codes = {
                1: "General error",
                9: "Emergency stop / Safety limits exceeded",
                11: "Not ready to move",
                12: "Not in position mode", 
                13: "Emergency IO triggered",
                14: "Emergency stop button pressed",
                19: "Joint limit exceeded",
                21: "Kinematics error",
                28: "Other error"
            }
            if ret in error_codes:
                print(f"Error meaning: {error_codes[ret]}")
            
            # Try to recover from error
            if ret == 9:
                print("Emergency stop triggered. Attempting recovery...")
                self.arm.clean_error()
                time.sleep(1.0)
                self.arm.motion_enable(enable=True)
                time.sleep(1.0)
                self.arm.set_state(state=0)
                time.sleep(1.0)
                print("Recovery attempt completed")
            return False
        else:
            print("Successfully moved to initialization pose")
        
        time.sleep(2.0)  # Allow time for movement to complete
        
        # Print pose after initialization
        print("\nAFTER initialization:")
        final_pos = self.print_current_pose()
        
        if final_pos is not None:
            # Check if orientation matches target
            orientation_error = [
                abs(final_pos[3] - target_orientation[0]),
                abs(final_pos[4] - target_orientation[1]), 
                abs(final_pos[5] - target_orientation[2])
            ]
            
            print(f"\nOrientation Error:")
            print(f"  Roll error: {orientation_error[0]:.4f} rad ({orientation_error[0]*180/3.14159:.2f} deg)")
            print(f"  Pitch error: {orientation_error[1]:.4f} rad ({orientation_error[1]*180/3.14159:.2f} deg)")
            print(f"  Yaw error: {orientation_error[2]:.4f} rad ({orientation_error[2]*180/3.14159:.2f} deg)")
            
            # Consider successful if error is less than 0.01 rad (~0.57 degrees)
            success = all(error < 0.01 for error in orientation_error)
            
            print(f"\nTest Result: {'PASSED' if success else 'FAILED'}")
            print("="*50)
            
            return success
        
        return False
    
    def close(self):
        """Disconnect from robot"""
        self.arm.disconnect()
        print("Disconnected from robot")

def main():
    try:
        # Initialize tester
        tester = TCPOrientationTester(
            ip="192.168.1.226",
            tcp_z_offset=218
        )
        
        # Test the TCP orientation initialization
        success = tester.initialize_tcp_orientation()
        
        if success:
            print("\n✅ TCP orientation initialization test PASSED!")
        else:
            print("\n❌ TCP orientation initialization test FAILED!")
        
        # Wait a moment before closing
        print("\nTest completed. Press Enter to exit...")
        input()
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"\nError occurred: {str(e)}")
        print(f"Error type: {type(e).__name__}")
        import traceback
        print("Full traceback:")
        traceback.print_exc()
    finally:
        if 'tester' in locals():
            print("\nClosing robot connection...")
            tester.close()
        print("Done.")

if __name__ == "__main__":
    main()