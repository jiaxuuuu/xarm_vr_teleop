#!/usr/bin/env python3
"""
Simple VR Bridge - minimal version using exact same pattern as robot_control.py
"""

import sys
import os
import json
import time
import numpy as np

# Add parent directory to path to find triad_openvr (same fix as robot_control.py)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import triad_openvr as vr
except ImportError as e:
    print(json.dumps({"error": f"Cannot import triad_openvr: {e}"}))
    sys.exit(1)

# Global variables (like robot_control.py)
controller = None
prev_pose_matrix = None

def initialize_vr():
    """Initialize VR system exactly like robot_control.py"""
    global controller
    
    try:
        print("Initializing VR system...", file=sys.stderr)
        v = vr.triad_openvr()
        print("VR system initialized successfully", file=sys.stderr)
        
        print("Connecting to controller...", file=sys.stderr)
        controller = v.devices["controller_1"]
        print("Controller connected successfully", file=sys.stderr)
        
        # Flush initial data (like robot_control.py)
        flush_controller_data()
        
        return True
        
    except Exception as e:
        print(f"VR initialization failed: {e}", file=sys.stderr)
        return False

def flush_controller_data(flush_count=50):
    """Flush initial controller data (from robot_control.py)"""
    global controller
    pose_matrix = controller.get_pose_matrix()
    while flush_count > 0:
        pose_matrix = controller.get_pose_matrix()
        if pose_matrix is None:
            continue
        flush_count -= 1

def get_controller_frame_change():
    """Get frame-to-frame controller changes"""
    global controller, prev_pose_matrix
    
    try:
        # Get current pose matrix (like robot_control.py)
        current_pose_matrix = controller.get_pose_matrix()
        if current_pose_matrix is None:
            return None
        
        # First frame - no change
        if prev_pose_matrix is None:
            prev_pose_matrix = current_pose_matrix
            return [0.0, 0.0, 0.0, 0.0]
        
        # Calculate position change
        current_pos = current_pose_matrix[:3, 3]
        prev_pos = prev_pose_matrix[:3, 3]
        pos_change = current_pos - prev_pos
        
        # Calculate rotation change (simplified - just Y axis for now)
        current_rot = current_pose_matrix[:3, :3]
        prev_rot = prev_pose_matrix[:3, :3]
        
        # Simple Y rotation extraction
        rel_rot = current_rot @ prev_rot.T
        y_rotation_change = np.arctan2(rel_rot[0, 2], rel_rot[2, 2])
        
        # Update previous state
        prev_pose_matrix = current_pose_matrix
        
        return [float(pos_change[0]), float(pos_change[1]), float(pos_change[2]), float(y_rotation_change)]
        
    except Exception as e:
        print(f"Error reading controller: {e}", file=sys.stderr)
        return None

def main():
    """Main loop"""
    if not initialize_vr():
        print(json.dumps({"error": "VR initialization failed"}))
        sys.exit(1)
    
    print(json.dumps({"status": "initialized"}))
    sys.stdout.flush()
    
    try:
        while True:
            # Read command from C++
            line = sys.stdin.readline().strip()
            if not line:
                break
                
            if line == "get_state":
                state = get_controller_frame_change()
                if state is not None:
                    result = {
                        "position_change": state[:3],
                        "y_rotation_change": state[3],
                        "should_exit": False
                    }
                else:
                    result = {"error": "Failed to read controller state"}
                    
                print(json.dumps(result))
                sys.stdout.flush()
                
            elif line == "exit":
                break
                
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(json.dumps({"error": str(e)}))

if __name__ == "__main__":
    main()