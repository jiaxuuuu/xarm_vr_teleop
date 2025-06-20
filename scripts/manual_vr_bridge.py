#!/usr/bin/env python3
"""
Manual VR Bridge - run in separate terminal
Outputs controller changes to stdout for C++ program to read
"""

import sys
import os
import json
import time
import numpy as np

# Add parent directory to path to find triad_openvr
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import triad_openvr as vr
    print("✅ Successfully imported triad_openvr", file=sys.stderr)
except ImportError as e:
    print(f"❌ Failed to import triad_openvr: {e}", file=sys.stderr)
    sys.exit(1)

# Global variables
controller = None
prev_pose_matrix = None

def initialize_vr():
    """Initialize VR system exactly like robot_control.py"""
    global controller
    
    try:
        print("🎮 Initializing VR system...", file=sys.stderr)
        v = vr.triad_openvr()
        print("✅ VR system initialized successfully", file=sys.stderr)
        
        print("🎯 Connecting to controller...", file=sys.stderr)
        controller = v.devices["controller_1"]
        print("✅ Controller connected successfully", file=sys.stderr)
        
        # Flush initial data (like robot_control.py)
        flush_controller_data()
        
        return True
        
    except Exception as e:
        print(f"❌ VR initialization failed: {e}", file=sys.stderr)
        return False

def flush_controller_data(flush_count=50):
    """Flush initial controller data (from robot_control.py)"""
    global controller
    print("🔄 Flushing initial controller data...", file=sys.stderr)
    for i in range(flush_count):
        pose_matrix = controller.get_pose_matrix()
        if pose_matrix is None:
            continue
    print("✅ Controller data flushed", file=sys.stderr)

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
            prev_pose_matrix = current_pose_matrix.copy()
            print("🔄 First frame - initializing with zero change", file=sys.stderr)
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
        
        # Debug output for significant changes
        change_magnitude = np.linalg.norm(pos_change)
        if change_magnitude > 0.001:  # > 1mm
            print(f"🎮 Controller change: pos=[{pos_change[0]*1000:.1f},{pos_change[1]*1000:.1f},{pos_change[2]*1000:.1f}]mm, rot={y_rotation_change*180/np.pi:.1f}°", file=sys.stderr)
        
        # Update previous state
        prev_pose_matrix = current_pose_matrix.copy()
        
        return [float(pos_change[0]), float(pos_change[1]), float(pos_change[2]), float(y_rotation_change)]
        
    except Exception as e:
        print(f"❌ Error reading controller: {e}", file=sys.stderr)
        return None

def main():
    """Main loop - continuously output controller state"""
    if not initialize_vr():
        print("❌ VR initialization failed - exiting", file=sys.stderr)
        sys.exit(1)
    
    print("🚀 VR Bridge ready! Move your controller...", file=sys.stderr)
    print("📊 Outputting controller data to stdout for C++ program", file=sys.stderr)
    
    try:
        frame_count = 0
        while True:
            state = get_controller_frame_change()
            if state is not None:
                result = {
                    "position_change": state[:3],
                    "y_rotation_change": state[3],
                    "should_exit": False,
                    "frame": frame_count
                }
                
                # Output JSON to stdout (for C++ program)
                print(json.dumps(result))
                sys.stdout.flush()
                
                frame_count += 1
                
            time.sleep(0.05)  # 20 Hz update rate
                
    except KeyboardInterrupt:
        print("\n🛑 VR Bridge stopped by user", file=sys.stderr)
    except Exception as e:
        print(f"❌ Error: {e}", file=sys.stderr)

if __name__ == "__main__":
    main()