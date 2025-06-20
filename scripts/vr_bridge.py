#!/usr/bin/env python3
"""
VR Bridge for C++ integration
Provides simple interface to read VR controller state for C++ teleoperation
"""

import sys
import os
import json
import time
import numpy as np

# Import triad_openvr from local directory (same way as robot_control.py)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import triad_openvr as vr
    print("Successfully imported triad_openvr", file=sys.stderr)
except ImportError as e:
    print(f"Failed to import triad_openvr: {e}", file=sys.stderr)
    print("Current path:", sys.path, file=sys.stderr)
    sys.exit(1)

class VRBridge:
    def __init__(self):
        self.vr_system = None
        self.controller = None
        self.prev_pose_matrix = None
        self.initialized = False
        self.frame_1_to_2 = None  # Coordinate transformation matrix
        
    def initialize(self):
        """Initialize VR system and controller"""
        try:
            print("Initializing VR system...", file=sys.stderr)
            self.vr_system = vr.triad_openvr()
            print("VR system initialized successfully", file=sys.stderr)
            
            print("Connecting to controller...", file=sys.stderr)
            self.controller = self.vr_system.devices["controller_1"]
            print("Controller connected successfully", file=sys.stderr)
            
            # Flush initial data
            self._flush_controller_data()
            
            # Set up coordinate transformation (simplified - assume VR world = robot base_link)
            # In your original code, this comes from get_transforms()
            # For now, use identity matrix (VR world frame = robot base_link frame)
            self.frame_1_to_2 = np.eye(4)
            
            self.initialized = True
            return True
            
        except Exception as e:
            print(f"VR initialization failed: {e}", file=sys.stderr)
            return False
    
    def _flush_controller_data(self, flush_count=50):
        """Flush initial controller data to get stable readings"""
        for _ in range(flush_count):
            pose_matrix = self.controller.get_pose_matrix()
            if pose_matrix is not None:
                continue
    
    def get_controller_state(self):
        """
        Get controller frame-to-frame changes (like original robot_control.py)
        Returns: [pos_change_x, pos_change_y, pos_change_z, y_rotation_change]
        """
        if not self.initialized:
            return None
            
        try:
            # Get controller pose matrix
            current_pose_matrix = self.controller.get_pose_matrix()
            if current_pose_matrix is None:
                return None
            
            # Transform to robot base frame (like original code)
            current_pose_robot = self.frame_1_to_2 @ current_pose_matrix
            
            # First frame - initialize with no change
            if self.prev_pose_matrix is None:
                self.prev_pose_matrix = current_pose_robot
                return [0.0, 0.0, 0.0, 0.0]  # No change on first frame
            
            # Calculate frame-to-frame position change
            current_pos = current_pose_robot[:3, 3]
            prev_pos = self.prev_pose_matrix[:3, 3]
            pos_change = current_pos - prev_pos
            
            # Calculate frame-to-frame rotation change (simplified Y-axis only)
            # Extract current and previous rotation matrices
            current_rot = current_pose_robot[:3, :3]
            prev_rot = self.prev_pose_matrix[:3, :3]
            
            # Calculate relative rotation matrix
            rel_rot = current_rot @ prev_rot.T
            
            # Extract Y-axis rotation change (simplified)
            # Using the fact that small rotations can be approximated
            # This is a simplified version - your original code is more complex
            y_rotation_change = np.arctan2(rel_rot[0, 2], rel_rot[2, 2])
            
            # Update previous state
            self.prev_pose_matrix = current_pose_robot
            
            return [float(pos_change[0]), float(pos_change[1]), float(pos_change[2]), float(y_rotation_change)]
            
        except Exception as e:
            print(f"Error reading controller state: {e}", file=sys.stderr)
            return None
    
    def should_exit(self):
        """Check if user wants to exit (e.g., via controller button)"""
        try:
            if not self.initialized:
                return False
                
            inputs = self.controller.get_controller_inputs()
            if inputs is None:
                return False
                
            # Exit on menu button press (button ID may vary)
            return inputs.get('menu_button', False)
            
        except:
            return False

def main():
    """Main loop for VR bridge - outputs JSON controller state"""
    bridge = VRBridge()
    
    if not bridge.initialize():
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
                state = bridge.get_controller_state()
                if state is not None:
                    result = {
                        "position_change": state[:3],
                        "y_rotation_change": state[3],
                        "should_exit": bridge.should_exit()
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
    
    print(json.dumps({"status": "shutdown"}), file=sys.stderr)

if __name__ == "__main__":
    main()