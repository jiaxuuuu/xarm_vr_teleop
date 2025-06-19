from xarm.wrapper import XArmAPI
import time

def reset_tcp_to_standard_pose():
    """Move TCP to standard downward-pointing pose"""
    
    # Connect to xArm
    arm = XArmAPI("192.168.1.226")
    arm.clean_warn()
    arm.clean_error()
    
    if arm.get_state()[0] != 0:
        print("Failed to connect to xArm")
        return
    
    # Enable motion
    arm.motion_enable(enable=True)
    arm.set_mode(0)  # Position mode
    arm.set_state(state=0)  # Ready state
    
    # Get current position
    code, current_pose = arm.get_position()
    if code != 0:
        print(f"Failed to get current position, code: {code}")
        return
    
    current_pos = current_pose[:3]  # Keep current position
    print(f"Current position: {current_pos}")
    print(f"Current RPY: {current_pose[3:6]}")
    
    # Set to standard downward orientation: [0, 0, 0] RPY
    standard_rpy = [0, 0, 0]  # Roll=0, Pitch=0, Yaw=0 (pointing down)
    
    print("Moving to standard downward pose...")
    code = arm.set_position(*current_pos, *standard_rpy, wait=True, is_radian=True)
    
    if code == 0:
        print("Successfully moved to standard pose!")
        
        # Verify new pose
        code, new_pose = arm.get_position()
        if code == 0:
            print(f"New RPY: {new_pose[3:6]}")
            import numpy as np
            import transforms3d as t3d
            tcp_matrix = t3d.euler.euler2mat(new_pose[3], new_pose[4], new_pose[5], 'rxyz')
            z_dir = tcp_matrix[:, 2]
            print(f"TCP Z direction: {z_dir}")
            if z_dir[2] < -0.8:
                print("✓ TCP is now pointing DOWN")
            else:
                print("⚠ TCP is not pointing down")
    else:
        print(f"Failed to move to standard pose, code: {code}")
    
    arm.disconnect()

if __name__ == "__main__":
    reset_tcp_to_standard_pose()