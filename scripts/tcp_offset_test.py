#!/usr/bin/env python3

from xarm.wrapper import XArmAPI
import time

def main():
    # Connect to xArm
    ip = "192.168.1.226"  # Same IP as in robot_control_yaw.py
    arm = XArmAPI(ip)
    
    print(f"Connecting to xArm at {ip}...")
    
    # Clear warnings and errors
    arm.clean_warn()
    arm.clean_error()
    
    # Enable motion
    ret = arm.motion_enable(enable=True)
    if ret != 0:
        print(f"Motion enable failed with code: {ret}")
        return
    
    # Set mode to position mode
    ret = arm.set_mode(0)
    if ret != 0:
        print(f"Set mode failed with code: {ret}")
        return
    
    # Set state to ready
    ret = arm.set_state(state=0)
    if ret != 0:
        print(f"Set state failed with code: {ret}")
        return
    
    print("Robot initialized successfully")
    
    # Set TCP offset to 218mm in Z direction
    tcp_offset = [0, 0, 218, 0, 0, 0]  # [x, y, z, roll, pitch, yaw]
    ret = arm.set_tcp_offset(tcp_offset)
    if ret != 0:
        print(f"Set TCP offset failed with code: {ret}")
    else:
        print(f"TCP offset set to: {tcp_offset}")
    
    # Read current orientation (and position)
    code, position = arm.get_position(is_radian=True)
    if code == 0:
        print(f"\nCurrent TCP Position (mm): x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}")
        print(f"Current TCP Orientation (rad): roll={position[3]:.3f}, pitch={position[4]:.3f}, yaw={position[5]:.3f}")
        print(f"Current TCP Orientation (deg): roll={position[3]*180/3.14159:.1f}, pitch={position[4]*180/3.14159:.1f}, yaw={position[5]*180/3.14159:.1f}")
    else:
        print(f"Failed to get position, code: {code}")
    
    # Disconnect
    arm.disconnect()
    print("\nDisconnected from robot")

if __name__ == "__main__":
    main()