from xarm.wrapper import XArmAPI
import numpy as np
import transforms3d as t3d

def check_tcp_apis():
    """Check different xArm APIs to get TCP pose"""
    
    # Connect to xArm
    arm = XArmAPI("192.168.1.226")
    arm.clean_warn()
    arm.clean_error()
    
    if arm.get_state()[0] != 0:
        print("Failed to connect to xArm")
        return
    
    print("=== Checking different TCP pose APIs ===\n")
    
    # Method 1: get_position() with degrees
    print("1. arm.get_position() [degrees]:")
    code, pose_deg = arm.get_position(is_radian=False)
    print(f"   Code: {code}")
    print(f"   Position (mm): {pose_deg[:3]}")
    print(f"   RPY (degrees): {pose_deg[3:6]}")
    
    # Method 2: get_position() with radians  
    print("\n2. arm.get_position(is_radian=True) [radians]:")
    code, pose_rad = arm.get_position(is_radian=True)
    print(f"   Code: {code}")
    print(f"   Position (mm): {pose_rad[:3]}")
    print(f"   RPY (radians): {pose_rad[3:6]}")
    print(f"   RPY (degrees): {np.array(pose_rad[3:6]) * 180 / np.pi}")
    
    # Method 3: get_position_aa() - axis-angle
    print("\n3. arm.get_position_aa(is_radian=True) [axis-angle]:")
    code, pose_aa = arm.get_position_aa(is_radian=True)
    print(f"   Code: {code}")
    print(f"   Position (mm): {pose_aa[:3]}")
    print(f"   Axis-Angle: {pose_aa[3:6]}")
    
    # Method 4: Check forward kinematics from joint angles
    print("\n4. Forward kinematics check:")
    code, joint_angles = arm.get_servo_angle(is_radian=True)
    if code == 0:
        print(f"   Joint angles (rad): {joint_angles}")
        code, fk_pose = arm.get_forward_kinematics(joint_angles, input_is_radian=True, return_is_radian=True)
        if code == 0:
            print(f"   FK Position (mm): {fk_pose[:3]}")
            print(f"   FK RPY (rad): {fk_pose[3:6]}")
            print(f"   FK RPY (deg): {np.array(fk_pose[3:6]) * 180 / np.pi}")
    
    # Convert RPY to rotation matrix and analyze
    if code == 0:
        print("\n=== TCP Orientation Analysis ===")
        rpy = pose_rad[3:6]
        R = t3d.euler.euler2mat(rpy[0], rpy[1], rpy[2], 'rxyz')
        
        print(f"Rotation Matrix:")
        print(R)
        print(f"X-axis direction: {R[:, 0]}")
        print(f"Y-axis direction: {R[:, 1]}")  
        print(f"Z-axis direction: {R[:, 2]}")
        
        # Check if Z points down
        z_dir = R[:, 2]
        if z_dir[2] < -0.8:
            orientation = "DOWN ✓"
        elif z_dir[2] > 0.8:
            orientation = "UP"
        else:
            orientation = "SIDE"
        print(f"TCP Z-axis pointing: {orientation}")
        
        # Check coordinate frame conventions
        print(f"\n=== Coordinate Frame Check ===")
        print(f"If base frame is: X=forward, Y=left, Z=up")
        print(f"Then TCP X points: {describe_direction(R[:, 0])}")
        print(f"Then TCP Y points: {describe_direction(R[:, 1])}")
        print(f"Then TCP Z points: {describe_direction(R[:, 2])}")
    
    arm.disconnect()

def describe_direction(vec):
    """Describe which direction a vector points in base frame"""
    x, y, z = vec
    desc = []
    
    if abs(x) > 0.5:
        desc.append("forward" if x > 0 else "backward")
    if abs(y) > 0.5:
        desc.append("left" if y > 0 else "right")
    if abs(z) > 0.5:
        desc.append("up" if z > 0 else "down")
        
    return " + ".join(desc) if desc else "unclear"

if __name__ == "__main__":
    check_tcp_apis()