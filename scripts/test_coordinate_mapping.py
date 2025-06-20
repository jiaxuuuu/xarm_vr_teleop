#!/usr/bin/env python3
"""
Test coordinate mapping without VR hardware
Simulates frame-to-frame controller changes to verify mapping is correct
"""

import sys
import os
import json
import time
import numpy as np

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

class TestVRBridge:
    def __init__(self):
        self.initialized = True
        self.frame_count = 0
        
    def get_controller_state(self):
        """Generate small test movements"""
        self.frame_count += 1
        
        # Generate small sinusoidal movements (1mm amplitude)
        t = self.frame_count * 0.1
        pos_change = [
            0.001 * np.sin(t * 0.5),      # X: 1mm sine wave
            0.001 * np.cos(t * 0.3),      # Y: 1mm cosine wave  
            0.0005 * np.sin(t * 0.2)      # Z: 0.5mm sine wave
        ]
        
        # Small rotation change (0.1 degree)
        y_rotation_change = 0.001 * np.sin(t * 1.0)  # ~0.06 degrees
        
        return [float(pos_change[0]), float(pos_change[1]), float(pos_change[2]), float(y_rotation_change)]
    
    def should_exit(self):
        return self.frame_count > 100  # Stop after 100 frames

def main():
    """Main loop for test VR bridge"""
    bridge = TestVRBridge()
    
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
                result = {
                    "position_change": state[:3],
                    "y_rotation_change": state[3],
                    "should_exit": bridge.should_exit()
                }
                    
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