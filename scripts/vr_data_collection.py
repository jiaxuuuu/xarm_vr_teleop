"""
VR Teleoperation Data Collection for xArm

Usage:
python vr_data_collection.py -o <demo_save_dir>

VR Controller movement:
- Move controller to move robot TCP position
- Rotate controller to control TCP orientation 
- Trigger button: gripper control
- Menu button: exit program

Recording control:
Press "C" to start recording episode
Press "S" to stop recording episode  
Press "Q" to exit program
Press "Backspace" to delete the previously recorded episode
"""

import time
import click
import numpy as np
import pickle
import os
import json
from datetime import datetime
import sys
import threading
import queue

# Add parent directory to path to find triad_openvr
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import triad_openvr as vr
from calibrated_transforms import get_transforms_calibrated as get_transforms
from xarm.wrapper import XArmAPI

# Keyboard input handling
try:
    import keyboard
    KEYBOARD_AVAILABLE = True
except ImportError:
    print("Warning: keyboard module not available. Install with: pip install keyboard")
    KEYBOARD_AVAILABLE = False

class DataCollectionEnv:
    def __init__(self, output_dir, robot_ip, frequency=10):
        self.output_dir = output_dir
        self.robot_ip = robot_ip
        self.frequency = frequency
        self.dt = 1.0 / frequency
        
        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        
        # Initialize robot
        self.arm = XArmAPI(robot_ip)
        self.arm.clean_warn()
        self.arm.clean_error()
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)  # Position mode
        self.arm.set_state(state=0)  # Ready state
        self.arm.set_tcp_offset([0, 0, 218, 0, 0, 0])  # 218mm TCP offset
        
        # Initialize VR
        self.vr_system = vr.triad_openvr()
        self.controller = self.vr_system.devices["controller_1"]
        
        # Get calibrated transforms
        self.frame_1_to_2, self.init_ori_in_1, self.init_ori_in_2 = get_transforms()
        
        # Recording state
        self.is_recording = False
        self.current_episode = []
        self.episode_count = 0
        self.episodes = []
        
        # Initialize poses
        self._init_poses()
        
        print("Data collection environment initialized!")
        print(f"Output directory: {output_dir}")
        print(f"Robot IP: {robot_ip}")
        print(f"Control frequency: {frequency} Hz")
        
    def __enter__(self):
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        
    def _init_poses(self):
        """Initialize VR controller and robot poses"""
        # Flush controller data
        for _ in range(50):
            pose_matrix = self.controller.get_pose_matrix()
            if pose_matrix is None:
                continue
                
        # Get initial poses
        pose_matrix = self.controller.get_pose_matrix()
        while pose_matrix is None:
            pose_matrix = self.controller.get_pose_matrix()
            time.sleep(0.01)
            
        self.init_controller_position = self.frame_1_to_2 @ pose_matrix[:3, 3]
        
        # Get initial robot position
        code, pos = self.arm.get_position()
        if code == 0:
            self.init_eef_pos = np.array(pos[:3]) / 1000  # mm to meters
            self.init_tcp_orientation = np.array([pos[3], pos[4], pos[5]])  # radians
        else:
            raise Exception(f"Failed to get initial robot position, code: {code}")
            
        # Use actual controller orientation at startup as reference
        self.init_ori_in_1 = pose_matrix[:3, :3]
        self.init_ori_in_2 = self.frame_1_to_2 @ self.init_ori_in_1
        
        print("Initial poses set successfully")
        
    def get_robot_state(self):
        """Get current robot state"""
        code, pos = self.arm.get_position(is_radian=True)
        if code != 0:
            return None
            
        code, joint_angles = self.arm.get_servo_angle(is_radian=True) 
        if code != 0:
            return None
            
        state = {
            'timestamp': time.time(),
            'tcp_pose': pos,  # [x, y, z, rx, ry, rz] in mm and radians
            'joint_angles': joint_angles,  # Joint angles in radians
            'tcp_position_m': np.array(pos[:3]) / 1000,  # TCP position in meters
            'tcp_orientation': np.array(pos[3:6])  # TCP orientation in radians
        }
        return state
        
    def get_vr_action(self):
        """Get target action from VR controller"""
        pose_matrix = self.controller.get_pose_matrix()
        if pose_matrix is None:
            return None
            
        # Get controller inputs
        controller_inputs = self.controller.get_controller_inputs()
        if controller_inputs is None:
            controller_inputs = {'trigger': 0, 'menu_button': False}
            
        # Calculate target position
        curr_pos_in_1 = pose_matrix[:3, 3]
        curr_pos_in_2 = self.frame_1_to_2 @ curr_pos_in_1
        controller_offset = curr_pos_in_2 - self.init_controller_position
        desired_eef_pos = self.init_eef_pos + controller_offset * 0.1  # Scale by 0.1
        
        # Calculate target orientation
        curr_ori_in_1 = pose_matrix[:3, :3]
        curr_ori_in_2 = self.frame_1_to_2 @ curr_ori_in_1
        rel_controller_rotation = self.init_ori_in_2.T @ curr_ori_in_2
        
        # Extract rotations using the working method
        x_axis_rotation = np.arctan2(rel_controller_rotation[1, 2], rel_controller_rotation[2, 2])
        y_axis_rotation = -np.arctan2(-rel_controller_rotation[0, 2], 
                                     np.sqrt(rel_controller_rotation[1, 2]**2 + rel_controller_rotation[2, 2]**2))
        z_axis_rotation = np.arctan2(rel_controller_rotation[0, 1], rel_controller_rotation[0, 0])
        
        # Apply to initial TCP orientation with scaling
        target_rpy = [
            self.init_tcp_orientation[0] + z_axis_rotation * 0.1,
            self.init_tcp_orientation[1] + x_axis_rotation * 0.1, 
            self.init_tcp_orientation[2] + y_axis_rotation * 0.1
        ]
        
        action = {
            'timestamp': time.time(),
            'target_tcp_position': desired_eef_pos,  # meters
            'target_tcp_orientation': target_rpy,    # radians
            'target_tcp_pose': np.concatenate([desired_eef_pos * 1000, target_rpy]),  # mm + radians
            'gripper_command': controller_inputs['trigger'],
            'controller_pose_matrix': pose_matrix,
            'controller_inputs': controller_inputs
        }
        return action
        
    def execute_action(self, action):
        """Execute action on robot"""
        if action is None:
            return False
            
        target_pose = action['target_tcp_pose']
        
        # Check if pose is reachable
        code, _ = self.arm.get_inverse_kinematics(target_pose, input_is_radian=True, return_is_radian=True)
        if code != 0:
            print(f"IK failed, code: {code}")
            return False
            
        # Execute pose command
        ret = self.arm.set_position(*target_pose[:3], *target_pose[3:], wait=False, is_radian=True)
        if ret != 0:
            print(f"Position command failed, code: {ret}")
            return False
            
        # Handle gripper
        if action['gripper_command'] > 0.5:
            self.arm.set_gripper_position(0, wait=False)  # Close
        else:
            self.arm.set_gripper_position(850, wait=False)  # Open
            
        return True
        
    def start_episode(self):
        """Start recording new episode"""
        if self.is_recording:
            print("Already recording!")
            return
            
        self.current_episode = []
        self.is_recording = True
        self.episode_start_time = time.time()
        print(f"Started recording episode {self.episode_count + 1}")
        
    def stop_episode(self):
        """Stop recording current episode"""
        if not self.is_recording:
            print("Not recording!")
            return
            
        self.is_recording = False
        
        if len(self.current_episode) > 0:
            episode_data = {
                'episode_id': self.episode_count,
                'start_time': self.episode_start_time,
                'end_time': time.time(),
                'duration': time.time() - self.episode_start_time,
                'data_points': len(self.current_episode),
                'trajectory': self.current_episode
            }
            
            self.episodes.append(episode_data)
            self.episode_count += 1
            
            # Save episode immediately
            self._save_episode(episode_data)
            print(f"Stopped recording. Episode {self.episode_count} saved with {len(self.current_episode)} data points")
        else:
            print("No data recorded, episode discarded")
            
        self.current_episode = []
        
    def drop_last_episode(self):
        """Delete the most recently recorded episode"""
        if len(self.episodes) == 0:
            print("No episodes to delete")
            return
            
        dropped_episode = self.episodes.pop()
        self.episode_count -= 1
        
        # Delete saved file
        episode_file = os.path.join(self.output_dir, f"episode_{dropped_episode['episode_id']:04d}.pkl")
        if os.path.exists(episode_file):
            os.remove(episode_file)
            
        print(f"Deleted episode {dropped_episode['episode_id']}")
        
    def record_data_point(self, state, action):
        """Record a single data point"""
        if not self.is_recording or state is None or action is None:
            return
            
        data_point = {
            'timestamp': time.time(),
            'state': state,
            'action': action,
            'episode_time': time.time() - self.episode_start_time
        }
        
        self.current_episode.append(data_point)
        
    def _save_episode(self, episode_data):
        """Save single episode to file"""
        filename = f"episode_{episode_data['episode_id']:04d}.pkl"
        filepath = os.path.join(self.output_dir, filename)
        
        with open(filepath, 'wb') as f:
            pickle.dump(episode_data, f)
            
    def save_all_data(self):
        """Save all collected data"""
        # Save metadata
        metadata = {
            'collection_date': datetime.now().isoformat(),
            'robot_ip': self.robot_ip,
            'frequency': self.frequency,
            'total_episodes': len(self.episodes),
            'calibration_transform': self.frame_1_to_2.tolist(),
            'init_controller_position': self.init_controller_position.tolist(),
            'init_eef_position': self.init_eef_pos.tolist(),
            'init_tcp_orientation': self.init_tcp_orientation.tolist()
        }
        
        metadata_file = os.path.join(self.output_dir, 'metadata.json')
        with open(metadata_file, 'w') as f:
            json.dump(metadata, f, indent=2)
            
        # Save episode summary
        summary_file = os.path.join(self.output_dir, 'episodes_summary.pkl')
        with open(summary_file, 'wb') as f:
            pickle.dump(self.episodes, f)
            
        print(f"Saved {len(self.episodes)} episodes to {self.output_dir}")
        
    def close(self):
        """Clean up resources"""
        self.save_all_data()
        self.arm.disconnect()
        print("Environment closed")

def keyboard_listener(key_queue):
    """Listen for keyboard input in separate thread"""
    if not KEYBOARD_AVAILABLE:
        return
        
    while True:
        try:
            event = keyboard.read_event()
            if event.event_type == keyboard.KEY_DOWN:
                key_queue.put(event.name)
        except:
            break

@click.command()
@click.option('--output', '-o', required=True, help="Directory to save demonstration dataset.")
@click.option('--robot_ip', '-ri', default="192.168.1.226", help="xArm's IP address")
@click.option('--frequency', '-f', default=10, type=float, help="Control frequency in Hz.")
def main(output, robot_ip, frequency):
    """Main data collection loop"""
    
    # Setup keyboard input
    key_queue = queue.Queue()
    if KEYBOARD_AVAILABLE:
        keyboard_thread = threading.Thread(target=keyboard_listener, args=(key_queue,), daemon=True)
        keyboard_thread.start()
        print("Keyboard controls available:")
    else:
        print("Keyboard controls not available. Using VR controller menu button to exit.")
        
    print("Controls:")
    print("  C - Start recording episode")
    print("  S - Stop recording episode") 
    print("  Q - Exit program")
    print("  Backspace - Delete last episode")
    print("  VR Menu Button - Exit program")
    
    dt = 1.0 / frequency
    
    try:
        with DataCollectionEnv(output, robot_ip, frequency) as env:
            print("\nReady for data collection!")
            
            t_start = time.monotonic()
            iter_idx = 0
            stop = False
            
            while not stop:
                loop_start = time.time()
                
                # Handle keyboard input
                while not key_queue.empty():
                    try:
                        key = key_queue.get_nowait()
                        if key == 'q':
                            stop = True
                            print("Exit requested")
                        elif key == 'c':
                            env.start_episode()
                        elif key == 's': 
                            env.stop_episode()
                        elif key == 'backspace':
                            env.drop_last_episode()
                    except queue.Empty:
                        break
                
                # Get current state
                state = env.get_robot_state()
                
                # Get VR action
                action = env.get_vr_action()
                
                # Check VR menu button for exit
                if action and action['controller_inputs']['menu_button']:
                    stop = True
                    print("VR menu button pressed - exiting")
                
                # Execute action
                if action:
                    env.execute_action(action)
                
                # Record data if recording
                env.record_data_point(state, action)
                
                # Status display
                status = f"\rEpisodes: {env.episode_count}"
                if env.is_recording:
                    status += f" | Recording: {len(env.current_episode)} points"
                else:
                    status += " | Not recording"
                    
                if state:
                    tcp_pos = state['tcp_position_m']
                    status += f" | TCP: [{tcp_pos[0]:.3f}, {tcp_pos[1]:.3f}, {tcp_pos[2]:.3f}]"
                    
                print(status, end='', flush=True)
                
                # Timing control
                loop_time = time.time() - loop_start
                if loop_time < dt:
                    time.sleep(dt - loop_time)
                    
                iter_idx += 1
                
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nShutting down...")

if __name__ == '__main__':
    main()