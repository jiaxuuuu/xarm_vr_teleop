#!/usr/bin/env python3

import pyrealsense2 as rs
import cv2
import numpy as np
import time
import threading
import os
from datetime import datetime

class RealSenseRecorder:
    def __init__(self, target_serial='327122078728'):
        self.target_serial = target_serial
        self.pipeline = None
        self.config = None
        self.video_writer = None
        self.recording = False
        self.frame_width = 1920
        self.frame_height = 1080
        self.fps = 30
        self.recording_thread = None
        
    def find_realsense_camera(self):
        """Find RealSense camera by serial number"""
        print(f"🔍 Searching for RealSense camera with serial: {self.target_serial}")
        
        # Create context to access connected devices
        ctx = rs.context()
        devices = ctx.query_devices()
        
        if len(devices) == 0:
            print("❌ No RealSense devices found")
            return False
        
        print(f"📷 Found {len(devices)} RealSense device(s):")
        
        for i, device in enumerate(devices):
            serial = device.get_info(rs.camera_info.serial_number)
            name = device.get_info(rs.camera_info.name)
            product_line = device.get_info(rs.camera_info.product_line)
            
            print(f"   Device {i}: {name} ({product_line})")
            print(f"   Serial: {serial}")
            
            if serial == self.target_serial:
                print(f"✅ Found target camera: {name} (Serial: {serial})")
                return True
            
        print(f"❌ Target camera with serial {self.target_serial} not found")
        print("Available serials:", [dev.get_info(rs.camera_info.serial_number) for dev in devices])
        return False
    
    def get_available_resolutions(self):
        """Get available color stream resolutions"""
        print("📐 Checking available resolutions...")
        
        ctx = rs.context()
        devices = ctx.query_devices()
        
        for device in devices:
            if device.get_info(rs.camera_info.serial_number) == self.target_serial:
                sensors = device.query_sensors()
                for sensor in sensors:
                    if sensor.is_color_sensor():
                        profiles = sensor.get_stream_profiles()
                        color_profiles = [p for p in profiles if p.stream_type() == rs.stream.color]
                        
                        print("   Available color resolutions:")
                        resolutions = set()
                        for profile in color_profiles:
                            vp = profile.as_video_stream_profile()
                            res = f"{vp.width()}x{vp.height()}@{vp.fps()}fps"
                            resolutions.add((vp.width(), vp.height(), vp.fps()))
                        
                        # Sort by resolution
                        for width, height, fps in sorted(resolutions, reverse=True):
                            print(f"     {width}x{height} @ {fps}fps")
                        
                        return sorted(resolutions, reverse=True)
        return []
    
    def initialize_camera(self):
        """Initialize RealSense camera with optimal settings"""
        print(f"📹 Initializing RealSense camera (Serial: {self.target_serial})...")
        
        # Create pipeline and config
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Enable device with specific serial number
        self.config.enable_device(self.target_serial)
        
        # Get available resolutions
        available_resolutions = self.get_available_resolutions()
        
        if available_resolutions:
            # Use highest available resolution
            best_width, best_height, best_fps = available_resolutions[0]
            print(f"🎯 Using best available resolution: {best_width}x{best_height} @ {best_fps}fps")
            
            self.frame_width = best_width
            self.frame_height = best_height
            self.fps = best_fps
        
        # Configure color stream
        self.config.enable_stream(
            rs.stream.color, 
            self.frame_width, 
            self.frame_height, 
            rs.format.bgr8, 
            self.fps
        )
        
        # Start pipeline
        try:
            profile = self.pipeline.start(self.config)
            
            # Get actual stream info
            color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
            actual_width = color_profile.width()
            actual_height = color_profile.height()
            actual_fps = color_profile.fps()
            
            print(f"✅ RealSense camera initialized:")
            print(f"   Resolution: {actual_width}x{actual_height}")
            print(f"   FPS: {actual_fps}")
            print(f"   Format: BGR8")
            
            # Update settings to actual values
            self.frame_width = actual_width
            self.frame_height = actual_height
            self.fps = actual_fps
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to start RealSense pipeline: {e}")
            return False
    
    def setup_video_writer(self, filename):
        """Setup video writer for MP4 output"""
        print(f"🎬 Setting up video writer for: {filename}")
        
        # Use H264 codec for better quality and compatibility
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        
        self.video_writer = cv2.VideoWriter(
            filename,
            fourcc,
            self.fps,
            (self.frame_width, self.frame_height)
        )
        
        if not self.video_writer.isOpened():
            raise Exception("Failed to create video writer")
        
        print(f"✅ Video writer ready: {filename}")
    
    def recording_loop(self):
        """Main recording loop running in separate thread"""
        frame_count = 0
        start_time = time.time()
        
        while self.recording:
            try:
                # Wait for frames
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                
                # Get color frame
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                
                # Convert to numpy array
                color_image = np.asanyarray(color_frame.get_data())
                
                # Add timestamp and recording indicator overlay
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                cv2.putText(color_image, f"REC {timestamp}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                cv2.putText(color_image, f"Frame: {frame_count}", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                
                # Write frame to video file
                self.video_writer.write(color_image)
                frame_count += 1
                
                # Print progress every second
                if frame_count % int(self.fps) == 0:
                    elapsed = time.time() - start_time
                    print(f"\r🔴 Recording... {elapsed:.1f}s ({frame_count} frames)", end="", flush=True)
                    
            except Exception as e:
                print(f"\n❌ Error in recording loop: {e}")
                break
        
        elapsed = time.time() - start_time
        print(f"\n✅ Recording completed: {elapsed:.1f}s, {frame_count} frames")
    
    def start_recording(self, filename):
        """Start recording video"""
        if self.recording:
            print("⚠️ Already recording!")
            return
        
        # Add .mp4 extension if not present
        if not filename.lower().endswith('.mp4'):
            filename += '.mp4'
        
        # Create full path
        full_path = os.path.abspath(filename)
        
        try:
            self.setup_video_writer(full_path)
            self.recording = True
            
            # Start recording in separate thread
            self.recording_thread = threading.Thread(target=self.recording_loop)
            self.recording_thread.start()
            
            print(f"🔴 Recording started: {full_path}")
            print("   Press ENTER to stop recording...")
            
        except Exception as e:
            print(f"❌ Failed to start recording: {e}")
            self.recording = False
    
    def stop_recording(self):
        """Stop recording video"""
        if not self.recording:
            print("⚠️ Not currently recording!")
            return
        
        print("\n🛑 Stopping recording...")
        self.recording = False
        
        # Wait for recording thread to finish
        if self.recording_thread:
            self.recording_thread.join()
        
        # Release video writer
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
        
        print("✅ Recording stopped and saved")
    
    def preview_camera(self):
        """Show RealSense camera preview window"""
        print("👁️ RealSense camera preview (press 'q' to close)...")
        
        try:
            while True:
                # Wait for frames
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                
                # Get color frame
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                
                # Convert to numpy array
                color_image = np.asanyarray(color_frame.get_data())
                
                # Add preview indicator
                cv2.putText(color_image, "REALSENSE PREVIEW - Press 'q' to close", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Show frame info
                height, width = color_image.shape[:2]
                cv2.putText(color_image, f"Resolution: {width}x{height}", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                cv2.imshow('RealSense Preview', color_image)
                
                # Exit on 'q' key
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        except Exception as e:
            print(f"❌ Preview error: {e}")
        
        cv2.destroyAllWindows()
    
    def get_camera_info(self):
        """Get detailed camera information"""
        print("📊 Camera Information:")
        
        ctx = rs.context()
        devices = ctx.query_devices()
        
        for device in devices:
            serial = device.get_info(rs.camera_info.serial_number)
            if serial == self.target_serial:
                print(f"   Name: {device.get_info(rs.camera_info.name)}")
                print(f"   Serial: {serial}")
                print(f"   Product Line: {device.get_info(rs.camera_info.product_line)}")
                print(f"   Firmware: {device.get_info(rs.camera_info.firmware_version)}")
                print(f"   USB Type: {device.get_info(rs.camera_info.usb_type_descriptor)}")
                break
    
    def cleanup(self):
        """Clean up resources"""
        if self.recording:
            self.stop_recording()
        
        if self.pipeline:
            self.pipeline.stop()
        
        cv2.destroyAllWindows()


def main():
    print("🎥 RealSense Camera Recorder")
    print("=" * 50)
    
    recorder = RealSenseRecorder(target_serial='327122078728')
    
    try:
        # Check if RealSense camera is available
        if not recorder.find_realsense_camera():
            print("❌ Target RealSense camera not found. Exiting.")
            return
        
        # Initialize camera
        if not recorder.initialize_camera():
            print("❌ Failed to initialize camera. Exiting.")
            return
        
        # Show camera info
        recorder.get_camera_info()
        
        # Show preview to confirm it's working
        print("\n📺 Would you like to preview the camera first? (y/n): ", end="")
        if input().lower() == 'y':
            recorder.preview_camera()
        
        while True:
            print("\n" + "=" * 50)
            print("🎬 RealSense Recording Menu:")
            print("1. Start new recording")
            print("2. Preview camera")
            print("3. Show camera info")
            print("4. Exit")
            
            choice = input("Choose option (1-4): ").strip()
            
            if choice == '1':
                # Get filename from user
                default_name = f"realsense_recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                print(f"\n📝 Enter filename (default: {default_name}): ", end="")
                filename = input().strip()
                
                if not filename:
                    filename = default_name
                
                # Start recording
                recorder.start_recording(filename)
                
                # Wait for user to press enter
                input()  # This will block until user presses enter
                
                # Stop recording
                recorder.stop_recording()
                
            elif choice == '2':
                recorder.preview_camera()
                
            elif choice == '3':
                recorder.get_camera_info()
                
            elif choice == '4':
                print("👋 Exiting...")
                break
                
            else:
                print("❌ Invalid choice. Please try again.")
    
    except KeyboardInterrupt:
        print("\n⏹️ Interrupted by user")
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        recorder.cleanup()
        print("🧹 Cleanup completed")


if __name__ == "__main__":
    main()