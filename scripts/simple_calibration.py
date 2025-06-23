#!/usr/bin/env python3

import numpy as np
import sys
import os
import time
import json

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import triad_openvr as vr
from xarm.wrapper import XArmAPI

def simple_calibration():
    print("=== Simple VR-Robot Calibration (Relative Positioning) ===")
    print("Instructions:")
    print("- You DON'T need to touch controller to TCP exactly")
    print("- Just maintain the SAME relative position for all points")
    print("- Example: Keep controller 10cm above TCP for all points")
    print("- Or keep controller 5cm to the left of TCP for all points")
    print("- Use MENU BUTTON to capture each point (not trigger)")
    
    # Connect to VR and Robot
    v = vr.triad_openvr()
    controller = v.devices["controller_1"]
    
    arm = XArmAPI("192.168.1.226")
    arm.clean_warn()
    arm.clean_error()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    
    print("✅ VR and Robot connected")
    
    # First, establish the relative offset
    print("\n=== Establish Relative Position ===")
    print("Position the VR controller relative to the robot TCP")
    print("(e.g., 10cm above, 5cm to left, etc.)")
    print("Remember this relative position - you'll use it for ALL points")
    input("Press Enter when you understand the relative position...")
    
    # Ask how many points to capture
    while True:
        try:
            num_points = int(input("\nHow many calibration points? (minimum 3, recommended 4-6): "))
            if num_points >= 3:
                break
            else:
                print("Need at least 3 points for calibration")
        except ValueError:
            print("Please enter a number")
    
    print(f"\nWill capture {num_points} calibration points")
    print("Tips for best calibration:")
    print("- Spread points across your workspace")
    print("- Make points form a 3D shape (not all in same plane)")
    print("- Use corners/edges of your workspace")
    print("- Maintain exact same relative position for all points")
    
    # Collect calibration points
    vr_points = []
    robot_points = []
    
    for i in range(num_points):
        print(f"\n--- Point {i+1}/{num_points} ---")
        print("1. Move robot TCP to a new position")
        print("2. Move VR controller to maintain the SAME relative position")
        print("   (e.g., if first point was 10cm above TCP, keep it 10cm above)")
        print("3. Press MENU button when both are positioned correctly")
        
        # Wait for menu button
        while True:
            inputs = controller.get_controller_inputs()
            if inputs and inputs.get('menu_button', False):
                break
            time.sleep(0.1)
        
        # Get positions
        vr_pose = controller.get_pose_matrix()
        while vr_pose is None:
            vr_pose = controller.get_pose_matrix()
            time.sleep(0.1)
        
        code, robot_pos = arm.get_position()
        if code != 0:
            print("❌ Failed to get robot position")
            continue
            
        vr_points.append(vr_pose[:3, 3])
        robot_points.append(np.array(robot_pos[:3]) / 1000)  # mm to m
        
        print(f"✅ Point {i+1} captured")
        print(f"   VR: [{vr_pose[:3, 3][0]:.3f}, {vr_pose[:3, 3][1]:.3f}, {vr_pose[:3, 3][2]:.3f}]")
        print(f"   Robot: [{robot_pos[0]/1000:.3f}, {robot_pos[1]/1000:.3f}, {robot_pos[2]/1000:.3f}]")
        
        # Wait for menu button release
        while inputs and inputs.get('menu_button', False):
            inputs = controller.get_controller_inputs()
            time.sleep(0.1)
    
    # Calculate transformation using all points (least squares approach)
    vr_points = np.array(vr_points)
    robot_points = np.array(robot_points)
    
    print(f"\n=== Calculating Transformation from {num_points} Points ===")
    
    # Calculate all relative vectors from first point
    vr_vectors = []
    robot_vectors = []
    
    for i in range(1, num_points):
        vr_vec = vr_points[i] - vr_points[0]
        robot_vec = robot_points[i] - robot_points[0]
        vr_vectors.append(vr_vec)
        robot_vectors.append(robot_vec)
        print(f"Vector {i}:")
        print(f"  VR: [{vr_vec[0]:.3f}, {vr_vec[1]:.3f}, {vr_vec[2]:.3f}]")
        print(f"  Robot: [{robot_vec[0]:.3f}, {robot_vec[1]:.3f}, {robot_vec[2]:.3f}]")
    
    vr_vectors = np.array(vr_vectors)
    robot_vectors = np.array(robot_vectors)
    
    # Use least squares to find transformation matrix
    # We want to solve: robot_vectors = frame_1_to_2 @ vr_vectors
    # Rearrange: robot_vectors.T = vr_vectors.T @ frame_1_to_2.T
    # So: frame_1_to_2.T = lstsq(vr_vectors, robot_vectors)
    # Therefore: frame_1_to_2 = lstsq(vr_vectors, robot_vectors).T
    
    try:
        # Least squares solution
        frame_1_to_2_T, residuals, rank, s = np.linalg.lstsq(vr_vectors, robot_vectors, rcond=None)
        frame_1_to_2 = frame_1_to_2_T.T
        
        print(f"\nLeast squares solution:")
        print(f"  Residuals: {residuals}")
        print(f"  Matrix rank: {rank}")
        print(f"  Condition number: {s[0]/s[-1]:.2f}" if len(s) > 1 else "")
        
        if rank < 3:
            print("⚠️ Warning: Underdetermined system (points may be coplanar)")
        
    except np.linalg.LinAlgError:
        print("❌ Least squares failed, using fallback method")
        
        # Fallback: use first two vectors if available
        if len(vr_vectors) >= 2:
            vr_vec1, vr_vec2 = vr_vectors[0], vr_vectors[1]
            robot_vec1, robot_vec2 = robot_vectors[0], robot_vectors[1]
            
            # Add third orthogonal vector
            vr_vec3 = np.cross(vr_vec1, vr_vec2)
            robot_vec3 = np.cross(robot_vec1, robot_vec2)
            
            if np.linalg.norm(vr_vec3) > 1e-6 and np.linalg.norm(robot_vec3) > 1e-6:
                vr_vec3 = vr_vec3 / np.linalg.norm(vr_vec3)
                robot_vec3 = robot_vec3 / np.linalg.norm(robot_vec3)
                
                VR_full = np.column_stack([vr_vec1, vr_vec2, vr_vec3])
                Robot_full = np.column_stack([robot_vec1, robot_vec2, robot_vec3])
                
                frame_1_to_2 = Robot_full @ np.linalg.inv(VR_full)
            else:
                frame_1_to_2 = np.eye(3)
        else:
            frame_1_to_2 = np.eye(3)
    
    print(f"\n✅ Calibration complete!")
    print("Transformation matrix:")
    print(frame_1_to_2)
    
    # Test accuracy using all relative vectors
    print(f"\n=== Testing Accuracy on {num_points-1} Vectors ===")
    errors = []
    
    for i in range(1, num_points):  # Test all vectors from point 0 to other points
        vr_vec = vr_points[i] - vr_points[0]
        robot_vec = robot_points[i] - robot_points[0]
        
        # Transform VR vector to robot frame
        transformed_vec = frame_1_to_2 @ vr_vec
        
        # Calculate error
        error = np.linalg.norm(transformed_vec - robot_vec)
        errors.append(error)
        
        print(f"Vector {i}:")
        print(f"  Robot actual: [{robot_vec[0]:.3f}, {robot_vec[1]:.3f}, {robot_vec[2]:.3f}]")
        print(f"  VR transformed: [{transformed_vec[0]:.3f}, {transformed_vec[1]:.3f}, {transformed_vec[2]:.3f}]")
        print(f"  Error: {error:.4f} m ({error*1000:.1f} mm)")
    
    avg_error = np.mean(errors) if errors else 0
    max_error = np.max(errors) if errors else 0
    print(f"\nCalibration Accuracy:")
    print(f"  Average error: {avg_error:.4f} m ({avg_error*1000:.1f} mm)")
    print(f"  Maximum error: {max_error:.4f} m ({max_error*1000:.1f} mm)")
    
    # Quality assessment
    if avg_error < 0.005:  # < 5mm
        print("✅ EXCELLENT calibration quality")
    elif avg_error < 0.010:  # < 10mm
        print("✅ GOOD calibration quality")
    elif avg_error < 0.020:  # < 20mm
        print("⚠️ ACCEPTABLE calibration quality")
    else:
        print("❌ POOR calibration quality - consider recalibrating")
    
    # Additional test: check if transformation preserves angles (if we have enough vectors)
    if len(vr_vectors) >= 2:
        vr_vec1, vr_vec2 = vr_vectors[0], vr_vectors[1]
        robot_vec1, robot_vec2 = robot_vectors[0], robot_vectors[1]
        
        vr_angle = np.arccos(np.clip(np.dot(vr_vec1, vr_vec2) / (np.linalg.norm(vr_vec1) * np.linalg.norm(vr_vec2)), -1, 1))
        robot_angle = np.arccos(np.clip(np.dot(robot_vec1, robot_vec2) / (np.linalg.norm(robot_vec1) * np.linalg.norm(robot_vec2)), -1, 1))
        angle_error = abs(vr_angle - robot_angle) * 180 / np.pi
        print(f"  Angle preservation error: {angle_error:.2f} degrees")
    
    # Save
    save_data = {
        "frame_1_to_2": frame_1_to_2.tolist(),
        "calibration_type": "relative_positioning",
        "avg_error_mm": avg_error * 1000,
        "test_points": {
            "vr_points": vr_points.tolist(),
            "robot_points": robot_points.tolist()
        }
    }
    
    with open("scripts/frame_transformation.json", "w") as f:
        json.dump(save_data, f, indent=2)
    
    print("✅ Saved to frame_transformation.json")
    
    # Cleanup
    arm.disconnect()

if __name__ == "__main__":
    simple_calibration()