#!/usr/bin/env python3

import cv2
import numpy as np
import time
import threading
import os
from datetime import datetime

class CameraRecorder:
    def __init__(self, target_serial='327122078728'):
        self.target_serial = target_serial
        self.camera = None
        self.video_writer = None
        self.recording = False
        self.frame_width = 1920
        self.frame_height = 1080
        self.fps = 30
        self.recording_thread = None
        
    def find_camera_by_serial(self):
        """Find camera device by serial number"""
        print(f"🔍 Searching for camera with serial number: {self.target_serial}")
        
        # Try different camera indices (0-10)
        for camera_index in range(10):
            try:
                cap = cv2.VideoCapture(camera_index)
                if not cap.isOpened():
                    continue
                
                # Try to get camera info - this might vary depending on your system
                # Some cameras expose serial number through backend properties
                backend_name = cap.getBackendName()
                print(f"   Checking camera {camera_index} (Backend: {backend_name})")
                
                # Read a test frame to see if camera works
                ret, frame = cap.read()
                if ret:
                    height, width = frame.shape[:2]
                    print(f"   Camera {camera_index}: Resolution {width}x{height}")
                    
                    # For now, we'll manually check if this looks like the right camera
                    # You might need to visually confirm which index corresponds to your camera
                    cap.release()
                    
                    # Ask user if this is the correct camera
                    response = input(f"   Is camera {camera_index} your target camera? (y/n): ").lower()
                    if response == 'y':
                        print(f"✅ Using camera {camera_index}")
                        return camera_index
                else:
                    cap.release()
                    
            except Exception as e:
                print(f"   Error checking camera {camera_index}: {e}")
                continue
        
        print(f"❌ Could not find camera with serial {self.target_serial}")
        return None
    
    def initialize_camera(self, camera_index):
        """Initialize the camera with optimal settings"""
        print(f"📹 Initializing camera {camera_index}...")
        
        self.camera = cv2.VideoCapture(camera_index)
        
        if not self.camera.isOpened():
            raise Exception(f"Failed to open camera {camera_index}")
        
        # Set camera properties for best quality
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.camera.set(cv2.CAP_PROP_FPS, self.fps)
        
        # Get actual settings
        actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
        
        print(f"✅ Camera initialized:")
        print(f"   Resolution: {actual_width}x{actual_height}")
        print(f"   FPS: {actual_fps}")
        
        # Update our settings to match actual camera capabilities
        self.frame_width = actual_width
        self.frame_height = actual_height
        if actual_fps > 0:
            self.fps = actual_fps
    
    def setup_video_writer(self, filename):
        """Setup video writer for MP4 output"""
        print(f"🎬 Setting up video writer for: {filename}")
        
        # Use MP4V codec for MP4 files
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
            ret, frame = self.camera.read()
            
            if not ret:
                print("❌ Failed to read frame from camera")
                break
            
            # Add timestamp overlay
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            cv2.putText(frame, f"REC {timestamp}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Write frame to video file
            self.video_writer.write(frame)
            frame_count += 1
            
            # Print progress every second
            if frame_count % int(self.fps) == 0:
                elapsed = time.time() - start_time
                print(f"\r🔴 Recording... {elapsed:.1f}s ({frame_count} frames)", end="", flush=True)
        
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
        """Show camera preview window"""
        print("👁️ Camera preview (press 'q' to close)...")
        
        while True:
            ret, frame = self.camera.read()
            if not ret:
                print("❌ Failed to read frame")
                break
            
            # Add preview indicator
            cv2.putText(frame, "PREVIEW - Press 'q' to close", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow('Camera Preview', frame)
            
            # Exit on 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cv2.destroyAllWindows()
    
    def cleanup(self):
        """Clean up resources"""
        if self.recording:
            self.stop_recording()
        
        if self.camera:
            self.camera.release()
        
        cv2.destroyAllWindows()


def main():
    print("🎥 Camera Recorder with Serial Number Detection")
    print("=" * 50)
    
    recorder = CameraRecorder(target_serial='327122078728')
    
    try:
        # Find the camera
        camera_index = recorder.find_camera_by_serial()
        if camera_index is None:
            print("❌ Camera not found. Exiting.")
            return
        
        # Initialize camera
        recorder.initialize_camera(camera_index)
        
        # Show preview to confirm it's the right camera
        print("\n📺 Would you like to preview the camera first? (y/n): ", end="")
        if input().lower() == 'y':
            recorder.preview_camera()
        
        while True:
            print("\n" + "=" * 50)
            print("🎬 Recording Menu:")
            print("1. Start new recording")
            print("2. Preview camera")
            print("3. Exit")
            
            choice = input("Choose option (1-3): ").strip()
            
            if choice == '1':
                # Get filename from user
                default_name = f"recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
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