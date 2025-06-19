import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import transforms3d as t3d
import triad_openvr as vr
from vr_test import get_transforms
from xarm.wrapper import XArmAPI

def plot_frame(ax, origin, rotation_matrix, scale=0.1, label="", colors=None):
    """Plot a coordinate frame with customizable colors"""
    if colors is None:
        colors = ['red', 'green', 'blue']
    
    x_axis = rotation_matrix[:, 0] * scale
    y_axis = rotation_matrix[:, 1] * scale  
    z_axis = rotation_matrix[:, 2] * scale
    
    ax.quiver(origin[0], origin[1], origin[2], 
              x_axis[0], x_axis[1], x_axis[2], 
              color=colors[0], arrow_length_ratio=0.1, linewidth=2, label=f'{label} X')
    ax.quiver(origin[0], origin[1], origin[2],
              y_axis[0], y_axis[1], y_axis[2], 
              color=colors[1], arrow_length_ratio=0.1, linewidth=2, label=f'{label} Y')
    ax.quiver(origin[0], origin[1], origin[2],
              z_axis[0], z_axis[1], z_axis[2], 
              color=colors[2], arrow_length_ratio=0.1, linewidth=2, label=f'{label} Z')

def calculate_controller_y_rotation(controller_ori_in_2, init_ori_in_2):
    """Calculate controller Y-axis rotation like in robot_control.py"""
    # Get current and initial controller Y axes in robot frame
    curr_y_axis = controller_ori_in_2[:, 1]
    init_y_axis = init_ori_in_2[:, 1]
    
    # Calculate rotation angle around controller Y axis
    curr_x_axis = controller_ori_in_2[:, 0]
    init_x_axis = init_ori_in_2[:, 0]
    
    # Project X axes onto plane perpendicular to Y axis to isolate Y-rotation
    curr_x_proj = curr_x_axis - np.dot(curr_x_axis, curr_y_axis) * curr_y_axis
    init_x_proj = init_x_axis - np.dot(init_x_axis, init_y_axis) * init_y_axis
    
    # Normalize projections
    if np.linalg.norm(curr_x_proj) > 0.1 and np.linalg.norm(init_x_proj) > 0.1:
        curr_x_proj = curr_x_proj / np.linalg.norm(curr_x_proj)
        init_x_proj = init_x_proj / np.linalg.norm(init_x_proj)
        
        # Calculate rotation angle around Y axis
        cos_angle = np.dot(curr_x_proj, init_x_proj)
        sin_angle = np.dot(np.cross(curr_x_proj, init_x_proj), curr_y_axis)
        y_rotation_angle = np.arctan2(sin_angle, cos_angle)
    else:
        y_rotation_angle = 0
        
    return y_rotation_angle

def calculate_tcp_pose_from_controller(controller_pose_matrix, frame_1_to_2, init_controller_position, 
                                     init_eef_pos, init_ori_in_2, current_tcp_rpy):
    """Calculate TCP pose from controller input using same logic as robot_control.py"""
    
    # Transform controller to robot base frame
    curr_ori_in_2 = frame_1_to_2 @ controller_pose_matrix[:3, :3]
    
    # Calculate controller Y rotation
    y_rotation_angle = calculate_controller_y_rotation(curr_ori_in_2, init_ori_in_2)
    
    # Calculate TCP orientation change
    tcp_yaw_change = y_rotation_angle
    max_change = np.pi/8  # ±22.5 degrees max per step
    smoothing = 0.3       # Apply 30% of change per step
    tcp_yaw_change = np.clip(tcp_yaw_change, -max_change, max_change)
    tcp_yaw_change *= smoothing
    
    # Apply to current TCP orientation
    calc_tcp_rpy = [current_tcp_rpy[0],                    # Keep current roll
                    current_tcp_rpy[1],                    # Keep current pitch  
                    current_tcp_rpy[2] + tcp_yaw_change]   # Add yaw change
    
    # Calculate TCP position
    curr_pos_in_1 = controller_pose_matrix[:3, 3]
    curr_pos_in_2 = frame_1_to_2 @ curr_pos_in_1
    rel_vr_pos = curr_pos_in_2 - init_controller_position
    calc_tcp_pos = init_eef_pos + rel_vr_pos * 0.5  # Same scaling as robot_control.py
    
    return calc_tcp_pos, calc_tcp_rpy, y_rotation_angle

def real_time_visualization():
    """Real-time visualization of controller input and calculated TCP pose"""
    print("Starting real-time visualization...")
    print("Controller: Yellow/Magenta/Lime")
    print("Current TCP: Red/Green/Blue") 
    print("Calculated TCP: Orange/Purple/Cyan")
    print("Press Ctrl+C to exit")
    
    # Initialize VR
    try:
        v = vr.triad_openvr()
        controller = v.devices["controller_1"]
    except Exception as e:
        print(f"Failed to initialize VR: {e}")
        return
    
    # Initialize xArm connection
    try:
        arm = XArmAPI("192.168.1.226")
        arm.clean_warn()
        arm.clean_error()
        
        if arm.get_state()[0] != 0:
            print("Failed to connect to xArm")
            return
            
        print("Connected to xArm successfully")
    except Exception as e:
        print(f"Failed to connect to xArm: {e}")
        return
    
    # Get coordinate transforms and initial poses
    frame_1_to_2, init_ori_in_1, init_ori_in_2 = get_transforms()
    
    # Wait for valid controller pose
    print("Waiting for valid controller pose...")
    pose_matrix = None
    while pose_matrix is None:
        pose_matrix = controller.get_pose_matrix()
        time.sleep(0.1)
    
    # Get initial positions
    init_controller_position = frame_1_to_2 @ pose_matrix[:3, 3]
    
    code, init_tcp_pose = arm.get_position(is_radian=True)
    if code != 0:
        print("Failed to get initial TCP pose")
        return
    init_eef_pos = np.array(init_tcp_pose[:3]) / 1000  # mm to meters
    
    print("Initialization complete!")
    
    # Set up the plot
    fig = plt.figure(figsize=(15, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    try:
        while True:
            ax.clear()
            
            # Get current controller pose
            pose_matrix = controller.get_pose_matrix()
            if pose_matrix is None:
                time.sleep(0.1)
                continue
            
            # Get current TCP pose
            code, current_tcp_pose = arm.get_position(is_radian=True)
            if code != 0:
                time.sleep(0.1)
                continue
            
            current_tcp_pos = np.array(current_tcp_pose[:3]) / 1000  # mm to m
            current_tcp_rpy = current_tcp_pose[3:6]
            
            # Calculate what TCP pose should be based on controller
            calc_tcp_pos, calc_tcp_rpy, y_rotation = calculate_tcp_pose_from_controller(
                pose_matrix, frame_1_to_2, init_controller_position, init_eef_pos, 
                init_ori_in_2, current_tcp_rpy)
            
            # Transform controller to robot base frame for display
            controller_pos_robot = frame_1_to_2 @ pose_matrix[:3, 3]
            controller_ori_robot = frame_1_to_2 @ pose_matrix[:3, :3]
            
            # Plot robot base frame
            plot_frame(ax, [0, 0, 0], np.eye(3), scale=0.15, label="Base", 
                      colors=['lightcoral', 'lightgreen', 'lightblue'])
            
            # Plot controller frame
            plot_frame(ax, controller_pos_robot, controller_ori_robot, scale=0.08, label="Controller", 
                      colors=['yellow', 'magenta', 'lime'])
            
            # Plot current TCP frame
            current_tcp_matrix = t3d.euler.euler2mat(current_tcp_rpy[0], current_tcp_rpy[1], current_tcp_rpy[2], 'rxyz')
            plot_frame(ax, current_tcp_pos, current_tcp_matrix, scale=0.12, label="Current TCP", 
                      colors=['red', 'green', 'blue'])
            
            # Plot calculated TCP frame (offset for visibility)
            calc_tcp_matrix = t3d.euler.euler2mat(calc_tcp_rpy[0], calc_tcp_rpy[1], calc_tcp_rpy[2], 'rxyz')
            calc_tcp_pos_offset = calc_tcp_pos + np.array([0.05, 0, 0])
            plot_frame(ax, calc_tcp_pos_offset, calc_tcp_matrix, scale=0.10, label="Calculated TCP", 
                      colors=['orange', 'purple', 'cyan'])
            
            # Set plot properties
            center = current_tcp_pos
            range_val = 0.3
            ax.set_xlim([center[0] - range_val, center[0] + range_val])
            ax.set_ylim([center[1] - range_val, center[1] + range_val])
            ax.set_zlim([center[2] - range_val, center[2] + range_val])
            
            ax.set_xlabel('X (forward)')
            ax.set_ylabel('Y (left)')
            ax.set_zlabel('Z (up)')
            ax.legend()
            
            # Title with current values
            ax.set_title(f"Controller Y: {y_rotation*180/np.pi:.1f}° | "
                        f"Current TCP: [{current_tcp_pos[0]:.3f}, {current_tcp_pos[1]:.3f}, {current_tcp_pos[2]:.3f}] | "
                        f"Calculated TCP: [{calc_tcp_pos[0]:.3f}, {calc_tcp_pos[1]:.3f}, {calc_tcp_pos[2]:.3f}]")
            
            plt.pause(0.1)
            
    except KeyboardInterrupt:
        print("\nVisualization stopped")
    finally:
        arm.disconnect()
        plt.close()

if __name__ == "__main__":
    real_time_visualization()