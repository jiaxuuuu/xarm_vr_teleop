#!/usr/bin/env python3
"""
Safe VR Bridge with extensive error handling
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
vr_system = None

def safe_get_pose_matrix():
    """Safely get pose matrix with error handling"""
    global controller
    
    try:
        if controller is None:
            return None
            
        pose_matrix = controller.get_pose_matrix()
        
        # Validate the matrix
        if pose_matrix is None:
            return None
            
        if not isinstance(pose_matrix, np.ndarray):
            return None
            
        if pose_matrix.shape != (4, 4):
            print(f"❌ Invalid pose matrix shape: {pose_matrix.shape}", file=sys.stderr)
            return None
            
        # Check for NaN or inf values
        if np.any(np.isnan(pose_matrix)) or np.any(np.isinf(pose_matrix)):
            print("❌ Pose matrix contains NaN or inf values", file=sys.stderr)
            return None
            
        return pose_matrix
        
    except Exception as e:
        print(f"❌ Error getting pose matrix: {e}", file=sys.stderr)
        return None

def initialize_vr():
    """Initialize VR system with extensive error handling"""
    global controller, vr_system
    
    try:
        print("🎮 Initializing VR system...", file=sys.stderr)
        vr_system = vr.triad_openvr()
        print("✅ VR system initialized successfully", file=sys.stderr)
        
        print("🎯 Connecting to controller...", file=sys.stderr)
        
        # Check if controller_1 exists
        if "controller_1" not in vr_system.devices:
            print("❌ controller_1 not found in VR devices", file=sys.stderr)
            print(f"Available devices: {list(vr_system.devices.keys())}", file=sys.stderr)
            return False
            
        controller = vr_system.devices["controller_1"]
        print("✅ Controller connected successfully", file=sys.stderr)
        
        # Test the controller
        print("🔍 Testing controller...", file=sys.stderr)
        test_pose = safe_get_pose_matrix()
        if test_pose is None:
            print("❌ Controller test failed - no pose data", file=sys.stderr)
            return False
            
        print("✅ Controller test passed", file=sys.stderr)
        
        # Flush initial data safely
        flush_controller_data()
        
        return True
        
    except Exception as e:
        print(f"❌ VR initialization failed: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc(file=sys.stderr)
        return False

def flush_controller_data(flush_count=20):
    """Safely flush initial controller data"""
    print("🔄 Flushing initial controller data...", file=sys.stderr)
    
    successful_reads = 0
    for i in range(flush_count * 2):  # Try more attempts
        pose_matrix = safe_get_pose_matrix()
        if pose_matrix is not None:
            successful_reads += 1
            
        time.sleep(0.01)  # Small delay
        
        if successful_reads >= flush_count:
            break
    
    if successful_reads < 5:
        print(f"⚠️  Only got {successful_reads} successful reads during flush", file=sys.stderr)
        return False
    
    print(f"✅ Controller data flushed ({successful_reads} successful reads)", file=sys.stderr)
    return True

def get_controller_frame_change():
    """Safely get frame-to-frame controller changes"""
    global controller, prev_pose_matrix
    
    try:
        # Get current pose matrix safely
        current_pose_matrix = safe_get_pose_matrix()
        if current_pose_matrix is None:
            return None
        
        # First frame - no change
        if prev_pose_matrix is None:
            prev_pose_matrix = current_pose_matrix.copy()
            print("🔄 First frame - initializing with zero change", file=sys.stderr)
            return [0.0, 0.0, 0.0, 0.0]
        
        # Calculate position change safely
        # VR world frame = robot base_link frame (direct mapping)
        try:
            current_pos = current_pose_matrix[:3, 3]
            prev_pos = prev_pose_matrix[:3, 3]
            pos_change = current_pos - prev_pos
            
            # Validate position change
            if np.any(np.isnan(pos_change)) or np.any(np.isinf(pos_change)):
                print("❌ Invalid position change", file=sys.stderr)
                return None
                
            # Safety limit: reject huge changes (likely errors)
            change_magnitude = np.linalg.norm(pos_change)
            if change_magnitude > 0.1:  # 10cm - definitely an error
                print(f"❌ Rejecting huge position change: {change_magnitude:.3f}m", file=sys.stderr)
                return None
                
        except Exception as e:
            print(f"❌ Error calculating position change: {e}", file=sys.stderr)
            return None
        
        # Calculate rotation change safely
        try:
            current_rot = current_pose_matrix[:3, :3]
            prev_rot = prev_pose_matrix[:3, :3]
            
            # Simple Y rotation extraction (safer than complex matrix operations)
            rel_rot = current_rot @ prev_rot.T
            y_rotation_change = np.arctan2(rel_rot[0, 2], rel_rot[2, 2])
            
            # Validate rotation change
            if np.isnan(y_rotation_change) or np.isinf(y_rotation_change):
                print("❌ Invalid rotation change", file=sys.stderr)
                y_rotation_change = 0.0
                
        except Exception as e:
            print(f"❌ Error calculating rotation change: {e}", file=sys.stderr)
            y_rotation_change = 0.0
        
        # Debug output for significant changes
        change_magnitude = np.linalg.norm(pos_change)
        if change_magnitude > 0.001:  # > 1mm
            print(f"🎮 Controller change: pos=[{pos_change[0]*1000:.1f},{pos_change[1]*1000:.1f},{pos_change[2]*1000:.1f}]mm, rot={y_rotation_change*180/np.pi:.1f}°", file=sys.stderr)
        
        # Update previous state safely
        prev_pose_matrix = current_pose_matrix.copy()
        
        return [float(pos_change[0]), float(pos_change[1]), float(pos_change[2]), float(y_rotation_change)]
        
    except Exception as e:
        print(f"❌ Error in get_controller_frame_change: {e}", file=sys.stderr)
        return None

def main():
    """Main loop with comprehensive error handling"""
    if not initialize_vr():
        print("❌ VR initialization failed - exiting", file=sys.stderr)
        sys.exit(1)
    
    print("🚀 VR Bridge ready! Move your controller...", file=sys.stderr)
    print("📊 Outputting controller data to stdout for C++ program", file=sys.stderr)
    
    frame_count = 0
    error_count = 0
    last_good_data_time = time.time()
    
    try:
        while True:
            try:
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
                    error_count = 0  # Reset error count on success
                    last_good_data_time = time.time()
                    
                else:
                    error_count += 1
                    if error_count > 10:
                        print(f"❌ Too many consecutive errors ({error_count})", file=sys.stderr)
                        break
                
                # Check for stale data
                if time.time() - last_good_data_time > 5.0:
                    print("❌ No good data for 5 seconds - controller disconnected?", file=sys.stderr)
                    break
                
                time.sleep(0.02)  # 50 Hz update rate (faster!)
                
            except KeyboardInterrupt:
                raise  # Re-raise to be caught by outer try
            except Exception as e:
                error_count += 1
                print(f"❌ Error in main loop: {e}", file=sys.stderr)
                if error_count > 20:
                    print("❌ Too many errors - exiting", file=sys.stderr)
                    break
                time.sleep(0.1)  # Wait a bit before retrying
                
    except KeyboardInterrupt:
        print("\n🛑 VR Bridge stopped by user", file=sys.stderr)
    except Exception as e:
        print(f"❌ Fatal error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc(file=sys.stderr)
    
    # Cleanup
    try:
        if vr_system:
            print("🔄 Cleaning up VR system...", file=sys.stderr)
            # Add any cleanup code here if needed
    except:
        pass
    
    print("✅ VR Bridge shutdown complete", file=sys.stderr)

if __name__ == "__main__":
    main()