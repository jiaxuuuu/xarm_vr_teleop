#!/usr/bin/env python3

import numpy as np
from calibrated_transforms import get_transforms_calibrated

def test_transformation():
    frame_1_to_2, init_ori_in_1, init_ori_in_2 = get_transforms_calibrated()
    
    print("=== Testing Coordinate Transformation ===")
    print("Calibrated transformation matrix (VR → Robot):")
    print(frame_1_to_2)
    print()
    
    # Test unit vectors in VR frame
    vr_unit_vectors = {
        "VR +X": np.array([1, 0, 0]),
        "VR +Y": np.array([0, 1, 0]), 
        "VR +Z": np.array([0, 0, 1])
    }
    
    print("VR unit vectors → Robot coordinates:")
    for name, vr_vec in vr_unit_vectors.items():
        robot_vec = frame_1_to_2 @ vr_vec
        print(f"{name:6} = {vr_vec} → {robot_vec} (magnitude: {np.linalg.norm(robot_vec):.3f})")
    
    print()
    print("Expected behavior:")
    print("- When you move controller in VR +X direction → Robot should move in direction:", frame_1_to_2 @ np.array([1, 0, 0]))
    print("- When you move controller in VR +Y direction → Robot should move in direction:", frame_1_to_2 @ np.array([0, 1, 0]))
    print("- When you move controller in VR +Z direction → Robot should move in direction:", frame_1_to_2 @ np.array([0, 0, 1]))

if __name__ == "__main__":
    test_transformation()