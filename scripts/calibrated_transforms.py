#!/usr/bin/env python3

import numpy as np
import json
import os

def get_transforms_calibrated():
    """
    Get VR to robot transformation using calibrated values.
    Falls back to default if calibration not found.
    """
    calibration_file = os.path.join(os.path.dirname(__file__), "frame_transformation.json")
    
    try:
        # Try to load calibrated transformation
        with open(calibration_file, 'r') as f:
            data = json.load(f)
        
        frame_1_to_2 = np.array(data["frame_1_to_2"])
        print("✅ Using calibrated VR-to-robot transformation")
        print("Calibration details:")
        if "avg_error_mm" in data:
            print(f"  Average error: {data['avg_error_mm']:.1f} mm")
        if "calibration_type" in data:
            print(f"  Type: {data['calibration_type']}")
        
    except FileNotFoundError:
        print("⚠️ No calibration found, using your specific calibrated matrix")
        print("Using the calibrated transformation from your session")
        
        # Your latest calibrated transformation (EXCELLENT quality, 2.1mm avg error)
        frame_1_to_2 = np.array([[ 0.42245838, -0.02309244, -0.90887107],
                                 [-0.8768039,  -0.03347457, -0.40844639],
                                 [-0.03463112,  1.04140466, -0.02522648]])
    
    except Exception as e:
        print(f"❌ Error loading calibration: {e}")
        print("Using your specific calibrated matrix as fallback")
        
        # Your latest calibrated transformation (EXCELLENT quality, 2.1mm avg error)
        frame_1_to_2 = np.array([[ 0.42245838, -0.02309244, -0.90887107],
                                 [-0.8768039,  -0.03347457, -0.40844639],
                                 [-0.03463112,  1.04140466, -0.02522648]])
    
    # For compatibility with existing code, we still need init orientations
    # Define initial controller orientation relative to robot base frame
    # This should align with how the controller is naturally held when robot is initialized
    init_ori_in_2 = np.array([[1, 0, 0],    # Controller X-axis aligns with robot X-axis
                              [0, 0, 1],    # Controller Y-axis aligns with robot Z-axis (up)  
                              [0, -1, 0]])  # Controller Z-axis aligns with robot -Y-axis
    
    # Calculate corresponding orientation in VR frame
    # init_ori_in_1 = frame_1_to_2^-1 @ init_ori_in_2
    init_ori_in_1 = np.linalg.inv(frame_1_to_2) @ init_ori_in_2
    
    return frame_1_to_2, init_ori_in_1, init_ori_in_2

if __name__ == "__main__":
    # Test the function
    frame_1_to_2, init_ori_in_1, init_ori_in_2 = get_transforms_calibrated()
    print("\nTransformation matrix (VR to Robot):")
    print(frame_1_to_2)
    print("\nInitial orientation in VR frame:")
    print(init_ori_in_1)
    print("\nInitial orientation in robot frame:")
    print(init_ori_in_2)