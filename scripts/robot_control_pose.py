import numpy as np
import sys
import os

# Add parent directory to path to find triad_openvr
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import triad_openvr as vr
import time
import transforms3d as t3d
from calibrated_transforms import get_transforms_calibrated as get_transforms
import matplotlib.pyplot as plt

# Import the official xArm SDK
from xarm.wrapper import XArmAPI

class XArmControl:
    def __init__(self, ip, mode=0, simulated=True, tcp_z_offset=188):
        self.arm = XArmAPI(ip)
        print(f"Connecting to xArm at {ip}...")
        
        # Clear warnings and errors
        self.arm.clean_warn()
        self.arm.clean_error()
        
        # Check connection and robot state
        if not self.check_connection():
            raise Exception("Failed to connect to xArm. Please check if the robot is powered on and connected.")
        
        # Print diagnostic information
        self.print_robot_state()
        
        # Initialize the robot
        self.initialize_robot(mode)
        
        # Set TCP offset
        self.arm.set_tcp_offset([0, 0, tcp_z_offset, 0, 0, 0])
        
        self.simulated = simulated
    
    def print_robot_state(self):
        """Print detailed robot state information"""
        print("\n=== Robot State Information ===")
        code, state = self.arm.get_state()
        
        states = {
            0: 'READY',
            1: 'SUSPENDED',
            2: 'STOPPED',
            3: 'PLAYING',
            4: 'PAUSED'
        }
        
        print(f"Current State: {states.get(state, f'UNKNOWN({state})')}")
        
        # Get current position
        code, pos = self.arm.get_position()
        if code == 0:
            print(f"Current Position (mm): x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}")
            if len(pos) > 3:
                print(f"Current Orientation (rad): rx={pos[3]:.2f}, ry={pos[4]:.2f}, rz={pos[5]:.2f}")
        
        # Get current joint angles
        code, angles = self.arm.get_servo_angle(is_radian=True)
        if code == 0:
            angles_deg = [angle * 180 / 3.14159 for angle in angles]
            print(f"Current Joint Angles (deg): {[f'{a:.2f}' for a in angles_deg]}")
        
        print("===========================\n")
    
    def check_connection(self):
        """Check if robot is connected properly"""
        state = self.arm.get_state()
        if state[0] == 0:  # Connected successfully
            return True
        return False
    
    def initialize_robot(self, mode):
        """Initialize robot with proper settings"""
        print("Initializing robot...")
        
        # Check current robot state first
        print("Checking current robot state...")
        code, state = self.arm.get_state()
        print(f"Current state code: {code}, state: {state}")
        
        # Clear errors and warnings
        print("Clearing errors and warnings...")
        self.arm.clean_error()
        self.arm.clean_warn()
        time.sleep(1.0)
        
        # Check if robot needs to be powered on
        print("Enabling motion...")
        ret = self.arm.motion_enable(enable=True)
        if ret != 0:
            print(f"Motion enable failed with code: {ret}")
            if ret == 9:
                print("Emergency stop is active! Please check robot safety systems.")
                raise Exception(f"Cannot enable motion due to emergency stop")
        print("Motion enabled successfully")
        time.sleep(1.0)
        
        # Set mode carefully
        print(f"Setting mode to {mode}...")
        ret = self.arm.set_mode(mode)
        if ret != 0:
            print(f"Set mode failed with code: {ret}")
            raise Exception(f"Failed to set robot mode, error code: {ret}")
        print(f"Mode set to {mode} successfully")
        time.sleep(1.0)
            
        # Set state to ready
        print("Setting state to READY...")
        ret = self.arm.set_state(state=0)
        if ret != 0:
            print(f"Set state failed with code: {ret}")
            raise Exception(f"Failed to set robot state, error code: {ret}")
        print("Robot state set to READY successfully")
        time.sleep(1.0)
        
        # Initialize TCP orientation to standard pose
        self.initialize_tcp_orientation()
        
        # Print final state
        print("Final robot state:")
        self.print_robot_state()
        
    def initialize_tcp_orientation(self):
        """Initialize robot to standard TCP orientation"""
        print("\nInitializing TCP orientation...")
        
        # Get current position
        code, current_pos = self.arm.get_position(is_radian=True)
        if code != 0:
            print(f"Failed to get current position, code: {code}")
            return
        
        # Set target orientation: roll=3.136, pitch=0.000, yaw=0.000
        target_orientation = [3.136, 0.000, 3.137]  # [roll, pitch, yaw] in radians
        
        # Keep current position, set target orientation
        target_pose = [current_pos[0], current_pos[1], current_pos[2]] + target_orientation
        
        print(f"Moving to initialization pose:")
        print(f"  Position (mm): x={target_pose[0]:.1f}, y={target_pose[1]:.1f}, z={target_pose[2]:.1f}")
        print(f"  Orientation (rad): roll={target_pose[3]:.3f}, pitch={target_pose[4]:.3f}, yaw={target_pose[5]:.3f}")
        
        # Check if target pose is reachable using inverse kinematics
        print("Checking if target pose is reachable...")
        code, joint_angles = self.arm.get_inverse_kinematics(target_pose, input_is_radian=True, return_is_radian=True)
        if code != 0:
            print(f"❌ Target pose is not reachable! IK failed with code: {code}")
            return
        else:
            print("✅ Target pose is reachable")
        
        # Reset robot state completely before movement to ensure clean state
        print("Resetting robot state before movement...")
        self.arm.clean_error()
        self.arm.clean_warn()
        time.sleep(1.0)
        
        # Re-enable motion and set state
        self.arm.motion_enable(enable=True)
        time.sleep(1.0)
        self.arm.set_mode(0)  # Position mode
        time.sleep(1.0)
        self.arm.set_state(state=0)  # Ready state
        time.sleep(1.0)
        
        # Use joint control with very slow movement to avoid emergency stops
        print("Moving to target orientation using slow joint control...")
        ret = self.arm.set_servo_angle(angle=joint_angles, speed=1, wait=True, is_radian=True)
        if ret != 0:
            print(f"Failed to move to initialization pose, code: {ret}")
            if not self.handle_error(ret, "initialize_tcp_orientation"):
                print("Initialization pose failed")
        else:
            print("Successfully moved to initialization pose")
        
        time.sleep(2.0)  # Allow time for movement to complete
        
        # Verify final orientation
        code, final_pos = self.arm.get_position(is_radian=True)
        if code == 0:
            final_orientation = final_pos[3:6]
            orientation_error = [abs(final_orientation[i] - target_orientation[i]) for i in range(3)]
            max_error = max(orientation_error)
            if max_error < 0.05:  # < 2.9 degrees
                print(f"✅ TCP orientation initialization successful")
                print(f"  Final orientation (rad): roll={final_orientation[0]:.3f}, pitch={final_orientation[1]:.3f}, yaw={final_orientation[2]:.3f}")
            else:
                print(f"⚠️ TCP orientation initialization completed with error: {max_error:.3f} rad ({max_error*180/3.14159:.1f} deg)")
        else:
            print("Could not verify final orientation")
    
    def handle_error(self, ret, operation=""):
        """Handle common error codes"""
        if ret == 9:  # Emergency stop
            print(f"\nEmergency stop detected during {operation}. Attempting to recover...")
            self.arm.clean_error()
            time.sleep(0.5)
            self.arm.motion_enable(enable=True)
            time.sleep(0.5)
            self.arm.set_state(state=0)
            time.sleep(0.5)
            self.print_robot_state()  # Print state after recovery attempt
            return True
        elif ret != 0:
            print(f"Error during {operation}, code: {ret}")
            return False
        return True
        
    def set_eef_position(self, x, y, z):
        # Convert to mm for xArm API
        pos = [x*1000, y*1000, z*1000]
        # Get current pose with explicit radian specification
        code, curr_pos = self.arm.get_position(is_radian=True)
        if code != 0:
            print(f"Failed to get current position for set_eef_position, code: {code}")
            return
        # Maintain current orientation - curr_pos[3:6] are in radians
        ret = self.arm.set_position(*pos, curr_pos[3], curr_pos[4], curr_pos[5], wait=True, is_radian=True)
        if not self.handle_error(ret, "set_eef_position"):
            print(f"Position command failed with code: {ret}")
        
    def set_eef_pose(self, x, y, z, roll=None, pitch=None, yaw=None):
        # Set both position and orientation
        pos = [x*1000, y*1000, z*1000]
        if roll is not None and pitch is not None and yaw is not None:
            # Set position and orientation - assume rpy values are in radians
            ret = self.arm.set_position(*pos, roll, pitch, yaw, wait=True, is_radian=True)
        else:
            # Maintain current orientation if not specified
            code, curr_pos = self.arm.get_position(is_radian=True)
            if code != 0:
                print(f"Failed to get current position for set_eef_pose, code: {code}")
                return
            ret = self.arm.set_position(*pos, curr_pos[3], curr_pos[4], curr_pos[5], wait=True, is_radian=True)
        if not self.handle_error(ret, "set_eef_pose"):
            print(f"Pose command failed with code: {ret}")
        
    def get_eef_position(self):
        # Convert from mm to meters
        ret, pos = self.arm.get_position()
        if ret == 0:
            return np.array(pos[:3]) / 1000
        self.handle_error(ret, "get_eef_position")
        return None
        
    def set_joint_velocity(self, velocities, duration=0.1):
        ret = self.arm.vc_set_joint_velocity(velocities, is_radian=True)
        if self.handle_error(ret, "set_joint_velocity"):
            time.sleep(duration)
        
    def open_gripper(self):
        ret = self.arm.set_gripper_position(850, wait=True)
        self.handle_error(ret, "open_gripper")
        
    def close_gripper(self):
        ret = self.arm.set_gripper_position(0, wait=True)
        self.handle_error(ret, "close_gripper")
        
    def reset(self):
        try:
            ret = self.arm.reset(wait=True)
            if ret == 0:
                print("Robot reset completed")
            elif ret == 9:
                print("Robot reset completed (emergency stop during reset is normal)")
            else:
                print(f"Robot reset completed with code: {ret}")
        except Exception as e:
            print(f"Reset completed with exception: {e}")
        
    def close(self):
        self.arm.disconnect()


def flush_controller_data(flush_count=50):
    pose_matrix = controller.get_pose_matrix()
    while flush_count > 0:
        pose_matrix = controller.get_pose_matrix()
        if pose_matrix is None:
            continue
        flush_count -= 1


def fetch_init_poses():
    global frame_1_to_2, init_ori_in_1, init_ori_in_2, init_controller_position, init_eef_pos, init_jangs

    flush_controller_data()
    frame_1_to_2, init_ori_in_1, init_ori_in_2 = get_transforms()

    pose_matrix = controller.get_pose_matrix()
    while pose_matrix is None:
        pose_matrix = controller.get_pose_matrix()

    init_controller_position = frame_1_to_2 @ pose_matrix[:3, 3]
    init_eef_pos = xarm.get_eef_position()
    
    # Use actual controller orientation at startup as reference
    init_ori_in_1 = pose_matrix[:3, :3]  # Real controller orientation in VR frame
    init_ori_in_2 = frame_1_to_2 @ init_ori_in_1  # Transform to robot frame
    
    code = 1
    while code != 0:
        code, init_jangs = xarm.arm.get_servo_angle(is_radian=True)


prev_rel_eef_pos = np.zeros(3)
rel_vr_pos_error_sum = 0
jang_error_sum = 0
pid_flush_count = 40
def get_eef_target_pos_ori(use_position_pid=False, ema_smooth_pos=False, use_jang_pid=False, dummy_ori=False, dummy_pos=False, duration=None, rpy_mask=[1,1,1]):
    global prev_rel_eef_pos, rel_vr_pos_error_sum, prev_actual_robot_jang, jang_error_sum, pid_flush_count

    pose_matrix = controller.get_pose_matrix()
    if pose_matrix is None:
        return None, None, None, None
    
    if dummy_ori:
        rpy = np.array([np.pi, 0, 0])
    else:
        # FULL POSE MAPPING: Map controller orientation directly to TCP orientation
        
        # Get current controller orientation in VR base frame
        curr_ori_in_1 = pose_matrix[:3, :3]
        
        # Transform current controller orientation to robot base frame
        curr_ori_in_2 = frame_1_to_2 @ curr_ori_in_1
        
        # Calculate the relative rotation from initial controller orientation
        rel_controller_rotation = init_ori_in_2.T @ curr_ori_in_2
        
        # Get the robot's initial TCP orientation (from initialization)
        init_tcp_orientation = np.array([3.136, 0.000, 3.137])  # [roll, pitch, yaw] in radians
        
        # Get current TCP pose for both position and orientation
        code, current_tcp_pose = xarm.arm.get_position(is_radian=True)
        if code == 0:
            current_tcp_pos = np.array(current_tcp_pose[:3]) / 1000  # mm to meters
            current_rpy = current_tcp_pose[3:6]  # Current [roll, pitch, yaw] in radians
        else:
            # Skip this iteration if we can't read current pose
            return None, None, None, None
        
        # FULL RPY MAPPING using the ORIGINAL working method extended to all axes
        # Extract rotations around each axis using the same method as the working yaw extraction
        
        # Roll rotation (around X-axis): Extract from Y-Z plane projection
        x_axis_rotation = np.arctan2(rel_controller_rotation[1, 2], rel_controller_rotation[2, 2])
        
        # Pitch rotation (around Y-axis): Extract from X-Z plane projection  
        y_axis_rotation_pitch = -np.arctan2(-rel_controller_rotation[0, 2], np.sqrt(rel_controller_rotation[1, 2]**2 + rel_controller_rotation[2, 2]**2))
        
        # Yaw rotation (around Z-axis): Extract from X-Y plane projection (ORIGINAL working method)
        z_axis_rotation = np.arctan2(rel_controller_rotation[0, 1], rel_controller_rotation[0, 0])
        
        # Apply relative changes to initial TCP orientation
        rpy = [init_tcp_orientation[0] + z_axis_rotation*0.15,     # Roll: around X-axis
               init_tcp_orientation[1] + x_axis_rotation*0.15,  # Pitch: around Y-axis  
               init_tcp_orientation[2] + y_axis_rotation_pitch*0.15]     # Yaw: around Z-axis (original working method)
        
        # Debug print - show all orientation mappings
        print(f"\rController RPY: R={x_axis_rotation*180/np.pi:.1f}° P={y_axis_rotation_pitch*180/np.pi:.1f}° Y={z_axis_rotation*180/np.pi:.1f}° -> TCP RPY: [{rpy[0]*180/np.pi:.1f}°, {rpy[1]*180/np.pi:.1f}°, {rpy[2]*180/np.pi:.1f}°]", end="")

    # Apply RPY mask to disable certain rotations if needed
    if not dummy_ori and not rpy_mask[0]:
        rpy[0] = current_rpy[0]  # Keep current roll when masked
    if not dummy_ori and not rpy_mask[1]:
        rpy[1] = current_rpy[1]  # Keep current pitch when masked
    if not dummy_ori and not rpy_mask[2]:
        rpy[2] = current_rpy[2]  # Keep current yaw when masked

    if dummy_pos:
        rel_vr_pos = init_eef_pos + np.array([duration/control_freq, 0, 0])
    else:
        # Relative position mapping: track position changes from initial position
        curr_pos_in_1 = pose_matrix[:3, 3]  # Current controller position in VR frame
        curr_pos_in_2 = frame_1_to_2 @ curr_pos_in_1  # Transform to robot base frame
        
        # Calculate controller movement relative to initial position (in robot base frame)
        controller_offset = curr_pos_in_2 - init_controller_position
        
        # Apply controller offset to initial TCP position (1:1 mapping in robot base frame)
        desired_eef_pos = init_eef_pos + controller_offset * 0.15  # Scale position changes by 0.5
        
        # Debug position changes
        if np.linalg.norm(controller_offset) > 0.001:  # > 1mm change
            print(f"\n[POS CHANGE] Controller offset: [{controller_offset[0]*1000:.1f}, {controller_offset[1]*1000:.1f}, {controller_offset[2]*1000:.1f}]mm -> TCP: [{desired_eef_pos[0]*1000:.1f}, {desired_eef_pos[1]*1000:.1f}, {desired_eef_pos[2]*1000:.1f}]mm", end="")

        Kp_jang =  0.1
        Ki_jang =  0.4 * 16
        PID_jang = None
        jang_code = 0
        if use_jang_pid:    
            jang_code, target_jang = xarm.arm.get_inverse_kinematics(np.concatenate([desired_eef_pos*1000, rpy]), input_is_radian=True, return_is_radian=True)
            if jang_code == 0:
                curr_jang_error = target_jang - prev_actual_robot_jang
                jang_error_sum += curr_jang_error
                error_I = jang_error_sum * (1/control_freq)
                PID_jang = Kp_jang * curr_jang_error + Ki_jang * error_I
            else:
                print(f"IK failed, code: {jang_code}")
    
    # Show calculated TCP pose in base frame (only when not dummy mode)
    if not dummy_pos and not dummy_ori:
        calc_pos_mm = desired_eef_pos * 1000  # Convert to mm
        calc_rpy_deg = np.array(rpy) * 180 / np.pi  # Convert to degrees
        print(f" | Calculated TCP: pos=[{calc_pos_mm[0]:.0f}, {calc_pos_mm[1]:.0f}, {calc_pos_mm[2]:.0f}]mm, rpy=[{calc_rpy_deg[0]:.0f}, {calc_rpy_deg[1]:.0f}, {calc_rpy_deg[2]:.0f}]°", end="")
                    
    return desired_eef_pos, rpy, PID_jang, jang_code


def recover_from_failure(start_pos, end_pos, maintain_rpy, use_position_pid):
    # start_point = np.concatenate([start_pos*1000, maintain_rpy])
    # end_point = np.concatenate([end_pos*1000, maintain_rpy])
    # code = xarm.arm.move_arc_lines([start_point, end_point], is_radian=True, wait=True)

    print(f"Recovering from failure! Hold")
    recovery_motion_size = 0.005
    step_size = recovery_motion_size/np.linalg.norm(start_pos - end_pos) 
    for t in np.arange(0, 1+step_size, step_size):
        loop_start_time = time.time()

        interm_pos = (1-t) * start_pos + t * end_pos
        code, curr_jangs = xarm.arm.get_inverse_kinematics(np.concatenate([interm_pos*1000, maintain_rpy]), input_is_radian=True, return_is_radian=True)
        if code != 0:
            continue
        
        # curr_jangs = 0.5*np.array(curr_jangs[:-1] + [0])
        # code = xarm.arm.set_servo_angle(angle=curr_jangs, wait=True, timeout=xarm_action_duration)
        code = xarm.arm.set_servo_angle_j(angles=curr_jangs, is_radian=True)
        controller.trigger_haptic_pulse()

        loop_duration = time.time()-loop_start_time
        # print(f"Loop duration: {loop_duration}")
        if loop_duration < xarm_action_duration:
            time.sleep(xarm_action_duration - loop_duration)

    rel_pos = None
    while rel_pos is None:
        rel_pos, rpy, _, _ = get_eef_target_pos_ori(dummy_ori=False, rpy_mask=[1,1,1], use_position_pid=use_position_pid)
    if np.linalg.norm(rel_pos - end_pos) > 0.01:
        recover_from_failure(end_pos, rel_pos, rpy, use_position_pid)


def robot_control_xarmapi(control_mode="joint_vel", use_position_pid=True, use_jang_pid=False):
    assert control_mode in ["eef_pos", "eef_pose", "eef_pose_vel", "joint_vel", "joint_ang"]
    global xarm_action_duration, prev_actual_robot_jang, prev_rel_eef_pos

    xarm_speed_lims = np.array(xarm.arm.get_ft_sensor_config()[1][21])*0.9

    start_time = time.time()
    duration = 0 
    prev_posori = np.zeros(6)
    prev_jangs = np.zeros(6)
    first = True

    if control_mode in ["eef_pos", "eef_pose_vel"]:
        xarm_action_duration = 0.3
    elif control_mode in ["joint_vel"]:
        xarm_action_duration = 0.2
    elif control_mode in ["joint_ang"]:
        xarm_action_duration = 0.05

    trajectory = []
    gripper_close_times = []
    gripper_open_times = []
    loop_count = 0
    recover_ikfail_from_pos = None
    recover_oob_from_pos = None
    gripper_closed = False
    eef_pos_target = None
    
    fetch_init_poses()
    prev_actual_robot_jang = np.array(init_jangs)

    while True:
        loop_start_time = time.time()

        if not (eef_pos_target is None):
            prev_valid_pos = eef_pos_target
        if control_mode == "eef_pos":
            # For position-only mode, use dummy orientation to avoid orientation calculations
            eef_pos_target, rpy, PID_jang, jang_code = get_eef_target_pos_ori(dummy_ori=True, rpy_mask=[1,1,1], use_position_pid=use_position_pid, use_jang_pid=use_jang_pid)
        else:
            # For pose mode, use FULL orientation calculations (all RPY enabled)
            eef_pos_target, rpy, PID_jang, jang_code = get_eef_target_pos_ori(dummy_ori=False, rpy_mask=[1,1,1], use_position_pid=use_position_pid, use_jang_pid=use_jang_pid)
        if eef_pos_target is None:
            controller.trigger_haptic_pulse()
            recover_oob_from_pos = prev_valid_pos
            continue
        if recover_oob_from_pos is not None and np.linalg.norm(eef_pos_target-recover_oob_from_pos) > 0.01:
            # recover_from_failure(recover_oob_from_pos, rel_pos, rpy, use_position_pid)
            # print("Recovered from Out of Bounds failure")
            recover_oob_from_pos = None
            print("\nController traveled too far while out of bounds for detection. Exiting to avoid potentially unsafe situation")
            break

        trajectory.append(eef_pos_target)

        # Setting eef pose with different control modes 
        if control_mode == "eef_pos":
            # Debug: Show position commands being sent
            pos_mm = eef_pos_target * 1000  # Convert to mm
            print(f"\n[POS CMD] Commanding position: [{pos_mm[0]:.0f}, {pos_mm[1]:.0f}, {pos_mm[2]:.0f}]mm", end="")
            xarm.set_eef_position(*eef_pos_target)
            print(f" -> SENT", end="")

        elif control_mode == "eef_pose":
            # Debug: Show pose commands being sent
            pos_mm = eef_pos_target * 1000  # Convert to mm
            rpy_deg = np.array(rpy) * 180 / np.pi  # Convert to degrees for display
            print(f"\n[POSE CMD] Commanding pose: pos=[{pos_mm[0]:.0f}, {pos_mm[1]:.0f}, {pos_mm[2]:.0f}]mm, rpy=[{rpy_deg[0]:.0f}, {rpy_deg[1]:.0f}, {rpy_deg[2]:.0f}]°", end="")
            xarm.set_eef_pose(eef_pos_target[0], eef_pos_target[1], eef_pos_target[2], 
                             rpy[0], rpy[1], rpy[2])
            print(f" -> SENT", end="")

        elif control_mode == "eef_pose_vel":
            curr_posori = np.concatenate([eef_pos_target*1000, rpy])
            curr_vel =  curr_posori - prev_posori
            prev_posori = curr_posori
            if first:
                first = False
                continue
            curr_vel = curr_vel.clip(-xarm_speed_lims, xarm_speed_lims)
            print(f"Setting {curr_vel}")
            code = xarm.arm.vc_set_cartesian_velocity(curr_vel, duration=xarm_action_duration, is_tool_coord=True)
            if code != 0:
                print(f"ERROR executing action, code: {code}")

        if use_jang_pid:
            curr_jangs = PID_jang
            code = jang_code
        else:
            code, curr_jangs = xarm.arm.get_inverse_kinematics(np.concatenate([eef_pos_target*1000, rpy]), input_is_radian=True, return_is_radian=True)
        
        if control_mode == "joint_vel":
            if code != 0:
                print(f"PID Calc Failed, skipping")
                continue
            curr_jangs = np.array(curr_jangs)
            curr_jvels = (curr_jangs[:6] - prev_jangs[:6]) * 1.5
            prev_jangs = curr_jangs
            if first:
                first = False
                continue
            code = xarm.set_joint_velocity(curr_jvels, duration=xarm_action_duration)
        
        elif control_mode == "joint_ang":
            if curr_jangs is None:
                print(f"\ncurr_jangs is None")
                continue
            if code != 0:
                print(f"\nIK Failed with code {code}, skipping")
                controller.trigger_haptic_pulse()
                recover_ikfail_from_pos = eef_pos_target
                continue

            if recover_ikfail_from_pos is not None:
                recover_from_failure(recover_ikfail_from_pos, eef_pos_target, rpy, use_position_pid)
                print("Recovered from IK failure")
                recover_ikfail_from_pos = None
                continue

            # Limit joint angle changes to prevent emergency stops
            curr_jangs = np.array(curr_jangs[:6])  # Only use first 6 joints
            if not first:
                # Get current joint angles
                code_current, current_jangs = xarm.arm.get_servo_angle(is_radian=True)
                if code_current == 0:
                    current_jangs = np.array(current_jangs[:6])
                    
                    # Limit maximum joint change per step
                    max_joint_change = 0.05  # ~2.9 degrees max per step
                    jang_diff = curr_jangs - current_jangs
                    jang_diff = np.clip(jang_diff, -max_joint_change, max_joint_change)
                    curr_jangs = current_jangs + jang_diff
                    
                    print(f"\n[JOINT LIMIT] Max change: {np.max(np.abs(jang_diff))*180/np.pi:.1f}°", end="")
            
            # Debug: Show when we're actually sending commands to robot
            curr_jangs_deg = curr_jangs * 180 / np.pi
            print(f"\n[ROBOT CMD] Sending joint angles: {curr_jangs_deg}", end="")
            
            code = xarm.arm.set_servo_angle_j(angles=curr_jangs, is_radian=True)
            if code != 0:
                print(f"\n[ERROR] Robot execution failed, code: {code}")
                if code == 9:
                    print("Emergency stop triggered!")
                    break
                elif code == 1:
                    print("Code 1 error - Motion blocked/Emergency stop! Stopping for safety.")
                    print(f"Last target pose: pos={eef_pos_target*1000}, rpy={np.array(rpy)*180/np.pi}")
                    # Immediately stop and reset robot
                    xarm.arm.set_state(state=4)  # Set to PAUSED state
                    break
                else:
                    print(f"Unknown error code {code}, stopping for safety")
                    break
            else:
                print(f" -> SUCCESS", end="")

        # Gripper action using trigger button
        controller_inputs = controller.get_controller_inputs()
        if controller_inputs['trigger'] > 0 and not gripper_closed:
            gripper_close_times.append(loop_count)
            xarm.close_gripper()
            gripper_closed = True
            print(f"Closing gripper")
        if controller_inputs['trigger'] == 0 and gripper_closed:
            gripper_open_times.append(loop_count)
            xarm.open_gripper()
            gripper_closed = False
            print(f"Opening gripper")
        loop_count += 1
        
        if controller_inputs["menu_button"]:
            break

        prev_rel_eef_pos = xarm.get_eef_position() - init_eef_pos
        code = 1
        while code != 0 and use_jang_pid:
            code, prev_actual_robot_jang = xarm.arm.get_servo_angle(is_radian=True)
            prev_actual_robot_jang = np.array(prev_actual_robot_jang)

        loop_duration = time.time()-loop_start_time
        # print(f"Loop duration: {loop_duration} = {1/loop_duration} Hz")
        # if loop_duration < xarm_action_duration:
        #     time.sleep(xarm_action_duration - loop_duration)
        # duration += xarm_action_duration
        if loop_duration < 1/control_freq:
            time.sleep(1/control_freq - loop_duration)
        duration += 1/control_freq
    xarm.reset()
    xarm.close()

    plt.figure("VR/EEF Trajectory")
    plt.plot(trajectory, label=['x','y','z'])
    plt.vlines(gripper_close_times, -0.5, 0.5, 'r', linestyles='dashed', label="Close gripper triggered")
    plt.vlines(gripper_open_times, -0.5, 0.5, 'g', linestyles='dashed', label="Open gripper triggered")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    # Control modes: "eef_pos" (position only), "eef_pose" (position + orientation), 
    # "joint_ang", "joint_vel", "eef_pose_vel"
    xarm_control_mode = "eef_pose"  # Full position + orientation mode
    simulated = False  # Set to True for simulation
    control_freq = 10  # Much lower frequency to prevent emergency stops

    if xarm_control_mode == "eef_pos":
        mode = 0
    elif xarm_control_mode == "eef_pose":
        mode = 0  # Same as position mode for xArm API
    elif xarm_control_mode == "joint_ang":
        mode = 1
    elif xarm_control_mode == "joint_vel":
        mode = 4
    elif xarm_control_mode == "eef_pose_vel":
        mode = 5

    try:
        print("Initializing VR system...")
        v = vr.triad_openvr()
        print("VR system initialized successfully")
        
        print("Connecting to controller...")
        controller = v.devices["controller_1"]
        print("Controller connected successfully")
        
        print("Initializing xArm...")
        xarm = XArmControl(
            ip="192.168.1.226", 
            mode=mode,
            simulated=simulated,
            tcp_z_offset=188
        )
        print("xArm initialized successfully")
        
        print("Starting robot control loop with FULL POSE MAPPING...")
        print("Controller orientation changes will map to ALL TCP rotations (Roll, Pitch, Yaw)")
        robot_control_xarmapi(control_mode=xarm_control_mode, use_position_pid=True)
    except KeyboardInterrupt:
        print("\nControl interrupted by user")
        if 'xarm' in locals():
            print("Emergency stopping robot at current pose...")
            xarm.arm.emergency_stop()  # Immediate stop
            xarm.arm.set_state(state=4)  # Set to PAUSED state
            print("Robot stopped")
            xarm.arm.disconnect()
            print("Robot disconnected")
    except Exception as e:
        print(f"\nError occurred: {str(e)}")
        print(f"Error type: {type(e).__name__}")
        import traceback
        print("Full traceback:")
        traceback.print_exc()
    finally:
        if 'xarm' in locals():
            print("\nClosing robot connection...")
            xarm.reset()
            xarm.close()
        print("Done.")