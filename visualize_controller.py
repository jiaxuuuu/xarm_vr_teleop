import numpy as np
import triad_openvr as vr
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
from vr_test import get_transforms

def plot_frame(ax, origin, rotation_matrix, scale=0.1, label="", colors=None):
    """Plot a coordinate frame with customizable colors"""
    if colors is None:
        colors = ['red', 'green', 'blue']  # Default X=red, Y=green, Z=blue
    
    # Define unit vectors for X, Y, Z axes
    x_axis = rotation_matrix[:, 0] * scale
    y_axis = rotation_matrix[:, 1] * scale  
    z_axis = rotation_matrix[:, 2] * scale
    
    # Plot axes
    ax.quiver(origin[0], origin[1], origin[2], 
              x_axis[0], x_axis[1], x_axis[2], 
              color=colors[0], arrow_length_ratio=0.1, linewidth=3, label=f'{label} X')
    ax.quiver(origin[0], origin[1], origin[2],
              y_axis[0], y_axis[1], y_axis[2], 
              color=colors[1], arrow_length_ratio=0.1, linewidth=3, label=f'{label} Y')
    ax.quiver(origin[0], origin[1], origin[2],
              z_axis[0], z_axis[1], z_axis[2], 
              color=colors[2], arrow_length_ratio=0.1, linewidth=3, label=f'{label} Z')

def visualize_controller_orientation():
    print("Starting VR controller orientation visualization...")
    print("Hold the controller in different orientations to see its frame.")
    print("Natural grip: X should point right, Y should point up, Z should point forward")
    print("Press Ctrl+C to exit")
    
    # Initialize VR
    v = vr.triad_openvr()
    controller = v.devices["controller_1"]
    
    # Get coordinate transforms
    frame_1_to_2, _, _ = get_transforms()
    
    # Set up the plot
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    try:
        while True:
            # Clear the plot
            ax.clear()
            
            # Get controller pose
            pose_matrix = controller.get_pose_matrix()
            if pose_matrix is None:
                time.sleep(0.1)
                continue
                
            # Extract position and orientation
            controller_pos_vr = pose_matrix[:3, 3]
            controller_ori_vr = pose_matrix[:3, :3]
            
            # Transform to robot base frame
            controller_pos_robot = frame_1_to_2 @ controller_pos_vr
            controller_ori_robot = frame_1_to_2 @ controller_ori_vr
            
            # Plot VR base frame at origin (gray colors)
            plot_frame(ax, [0, 0, 0], np.eye(3), scale=0.2, label="VR_Base", 
                      colors=['lightcoral', 'lightgreen', 'lightblue'])
            
            # Plot controller frame in robot space (bright colors)
            plot_frame(ax, controller_pos_robot, controller_ori_robot, scale=0.15, label="Controller", 
                      colors=['red', 'green', 'blue'])
            
            # Plot ideal TCP frame (Z pointing down) - different colors
            tcp_ideal_pos = controller_pos_robot + np.array([0.05, 0, 0])  # Offset slightly for visibility
            tcp_ideal_ori = np.array([[1, 0, 0],    # X forward
                                     [0, 1, 0],    # Y left  
                                     [0, 0, -1]])  # Z down
            plot_frame(ax, tcp_ideal_pos, tcp_ideal_ori, scale=0.12, label="TCP_Ideal", 
                      colors=['orange', 'purple', 'cyan'])
            
            # Set plot properties
            ax.set_xlim([-0.5, 0.5])
            ax.set_ylim([-0.5, 0.5]) 
            ax.set_zlim([-0.5, 0.5])
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.legend()
            
            # Print controller orientation info
            controller_rpy = np.array([
                np.arctan2(controller_ori_robot[2,1], controller_ori_robot[2,2]),
                np.arctan2(-controller_ori_robot[2,0], 
                          np.sqrt(controller_ori_robot[2,1]**2 + controller_ori_robot[2,2]**2)),
                np.arctan2(controller_ori_robot[1,0], controller_ori_robot[0,0])
            ]) * 180 / np.pi
            
            ax.set_title(f"Controller RPY: [{controller_rpy[0]:.1f}°, {controller_rpy[1]:.1f}°, {controller_rpy[2]:.1f}°]")
            
            # Show frame axes directions
            x_dir = controller_ori_robot[:, 0]
            y_dir = controller_ori_robot[:, 1] 
            z_dir = controller_ori_robot[:, 2]
            
            print(f"\rController frame directions: X={x_dir}, Y={y_dir}, Z={z_dir}", end="")
            
            plt.pause(0.1)
            
    except KeyboardInterrupt:
        print("\nVisualization stopped")
    finally:
        plt.close()

if __name__ == "__main__":
    visualize_controller_orientation()