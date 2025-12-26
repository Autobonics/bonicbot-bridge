#!/usr/bin/env python3
"""
Live Camera Viewer
Real-time camera stream viewer using OpenCV
"""

import argparse
import time
import cv2
from bonicbot_bridge import BonicBot

class LiveCameraViewer:
    def __init__(self):
        self.frame_count = 0
        self.start_time = time.time()
        self.latest_frame = None
        self.running = True
        
    def process_frame(self, image):
        """Callback for each camera frame"""
        self.frame_count += 1
        self.latest_frame = image.copy()
        
    def calculate_fps(self):
        """Calculate current FPS"""
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            return self.frame_count / elapsed
        return 0.0
    
    def add_overlay(self, frame):
        """Add FPS and info overlay to frame"""
        if frame is None:
            return None
            
        # Create copy for overlay
        display_frame = frame.copy()
        
        # Calculate FPS
        fps = self.calculate_fps()
        
        # Add black background for text
        cv2.rectangle(display_frame, (5, 5), (250, 65), (0, 0, 0), -1)
        
        # Add text overlay
        cv2.putText(display_frame, f"FPS: {fps:.1f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(display_frame, f"Frames: {self.frame_count}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        return display_frame
    
    def run(self, host, port=9090, window_name="BonicBot Live Camera"):
        """Run the live viewer"""
        print("=" * 60)
        print("🎥 BonicBot Live Camera Viewer")
        print("=" * 60)
        print(f"\nConnecting to robot at {host}:{port}...")
        
        try:
            with BonicBot(host=host, port=port) as bot:
                print("✓ Connected!")
                
                # Start camera service
                print("Starting camera...")
                bot.system.start_camera()
                time.sleep(0.5)
                
                # Start streaming with callback
                print("Starting live stream...")
                bot.start_camera(callback=self.process_frame)
                
                # Wait for first frame
                print("Waiting for frames...")
                timeout = time.time() + 5
                while self.latest_frame is None and time.time() < timeout:
                    time.sleep(0.1)
                
                if self.latest_frame is None:
                    print("❌ No frames received! Check camera.")
                    return
                
                print("\n✓ Stream started!")
                print("\nControls:")
                print("  - Press 'q' or ESC to quit")
                print("  - Press 's' to save snapshot")
                print("  - Press 'f' to toggle fullscreen")
                print()
                
                # Create window
                cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                fullscreen = False
                snapshot_count = 0
                
                # Main display loop
                while self.running:
                    if self.latest_frame is not None:
                        # Add overlay
                        display_frame = self.add_overlay(self.latest_frame)
                        
                        # Display frame
                        cv2.imshow(window_name, display_frame)
                    
                    # Handle keyboard input
                    key = cv2.waitKey(1) & 0xFF
                    
                    if key == ord('q') or key == 27:  # 'q' or ESC
                        print("\n👋 Stopping stream...")
                        break
                        
                    elif key == ord('s'):  # Save snapshot
                        if self.latest_frame is not None:
                            snapshot_count += 1
                            filename = f"snapshot_{snapshot_count:03d}.jpg"
                            cv2.imwrite(filename, self.latest_frame)
                            print(f"📸 Snapshot saved: {filename}")
                    
                    elif key == ord('f'):  # Toggle fullscreen
                        fullscreen = not fullscreen
                        if fullscreen:
                            cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN,
                                                cv2.WINDOW_FULLSCREEN)
                        else:
                            cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN,
                                                cv2.WINDOW_NORMAL)
                
                # Cleanup
                print("Stopping camera...")
                bot.stop_camera()
                bot.system.stop_camera()
                cv2.destroyAllWindows()
                
                # Statistics
                print("\n" + "=" * 60)
                print("📊 Session Statistics")
                print("=" * 60)
                print(f"Total frames: {self.frame_count}")
                print(f"Average FPS: {self.calculate_fps():.1f}")
                print(f"Duration: {time.time() - self.start_time:.1f}s")
                print(f"Snapshots saved: {snapshot_count}")
                print("=" * 60)
                
        except KeyboardInterrupt:
            print("\n\n⚠️  Interrupted by user")
            cv2.destroyAllWindows()
            
        except Exception as e:
            print(f"\n❌ Error: {e}")
            cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(description='Live camera viewer for BonicBot')
    parser.add_argument('--host', type=str, required=True, help='Robot IP address')
    parser.add_argument('--port', type=int, default=9090, help='ROS bridge port (default: 9090)')
    parser.add_argument('--window', type=str, default='BonicBot Live Camera',
                       help='Window name (default: "BonicBot Live Camera")')
    args = parser.parse_args()
    
    viewer = LiveCameraViewer()
    viewer.run(args.host, args.port, args.window)

if __name__ == '__main__':
    main()
