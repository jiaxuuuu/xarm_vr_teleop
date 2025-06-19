import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import transforms3d as t3d
from xarm.wrapper import XArmAPI

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

def visualize_tcp_orientation():
    print("Starting TCP orientation visualization...")
    print("This will show the current TCP frame orientation in real-time")
    print("TCP Frame: X=forward, Y=left, Z=up (or down when inverted)")
    print("Press Ctrl+C to exit")
    
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
    
    # Set up the plot
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    try:
        while True:
            # Clear the plot
            ax.clear()
            
            # Get current TCP pose using the correct API
            code, tcp_pose = arm.get_position(is_radian=True)
            
            if code != 0:
                print(f"Failed to get TCP position, code: {code}")
                time.sleep(0.1)
                continue
                
            # Extract position and orientation
            tcp_pos = np.array(tcp_pose[:3]) / 1000  # Convert mm to meters
            tcp_rpy = tcp_pose[3:6]  # [roll, pitch, yaw] in radians
            
            # Convert RPY to rotation matrix
            tcp_rotation_matrix = t3d.euler.euler2mat(tcp_rpy[0], tcp_rpy[1], tcp_rpy[2], 'rxyz')
            
            # Plot robot base frame at origin
            plot_frame(ax, [0, 0, 0], np.eye(3), scale=0.15, label="Base", 
                      colors=['lightcoral', 'lightgreen', 'lightblue'])
            
            # Plot current TCP frame - just show what we get
            plot_frame(ax, tcp_pos, tcp_rotation_matrix, scale=0.12, label="TCP", 
                      colors=['red', 'green', 'blue'])
            
            # Set plot properties
            center = tcp_pos
            range_val = 0.3
            ax.set_xlim([center[0] - range_val, center[0] + range_val])
            ax.set_ylim([center[1] - range_val, center[1] + range_val])
            ax.set_zlim([center[2] - range_val, center[2] + range_val])
            ax.set_xlabel('X (forward)')
            ax.set_ylabel('Y (left)')
            ax.set_zlabel('Z (up)')
            ax.legend()
            
            # Show TCP information
            tcp_rpy_deg = np.array(tcp_rpy) * 180 / np.pi
            ax.set_title(f"Current TCP Pose")
            
            # Print current TCP state
            x_dir = tcp_rotation_matrix[:, 0]  # TCP X axis direction
            y_dir = tcp_rotation_matrix[:, 1]  # TCP Y axis direction  
            z_dir = tcp_rotation_matrix[:, 2]  # TCP Z axis direction
            
            print(f"\rPosition (m): [{tcp_pos[0]:.3f}, {tcp_pos[1]:.3f}, {tcp_pos[2]:.3f}]", end="")
            print(f" | RPY (deg): [{tcp_rpy_deg[0]:.1f}, {tcp_rpy_deg[1]:.1f}, {tcp_rpy_deg[2]:.1f}]", end="")
            print(f" | Z-axis: [{z_dir[0]:.2f}, {z_dir[1]:.2f}, {z_dir[2]:.2f}]", end="")
            
            plt.pause(0.1)
            
    except KeyboardInterrupt:
        print("\nVisualization stopped")
    finally:
        arm.disconnect()
        plt.close()

if __name__ == "__main__":
    visualize_tcp_orientation()