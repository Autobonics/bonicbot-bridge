#!/usr/bin/env python3
"""
Camera Test Script
Tests camera streaming functionality of the BonicBot bridge library
"""

import argparse
import time
from bonicbot_bridge import BonicBot

def main():
    parser = argparse.ArgumentParser(description='Test BonicBot camera functionality')
    parser.add_argument('--host', type=str, required=True, help='Robot IP address')
    parser.add_argument('--port', type=int, default=9090, help='ROS bridge port (default: 9090)')
    parser.add_argument('--output', type=str, default='robot_view.jpg', help='Output image filename')
    args = parser.parse_args()
    
    print("="*60)
    print("BonicBot Camera Test")
    print("="*60)
    
    # Connect to robot
    print(f"\n1. Connecting to robot at {args.host}:{args.port}...")
    with BonicBot(host=args.host, port=args.port) as robot:
        print("   ✓ Connected!")
        
        # Start camera service
        print("\n2. Starting camera service...")
        robot.system.start_camera()
        time.sleep(1)
        
        # Start streaming
        print("\n3. Starting camera stream...")
        
        frame_count = 0
        def image_callback(image):
            nonlocal frame_count
            frame_count += 1
            if frame_count % 5 == 0:
                print(f"   📷 Received frame {frame_count}, shape: {image.shape}")
        
        robot.start_camera(callback=image_callback)
        
        # Wait for camera info
        robot.camera.wait_for_image(timeout=5.0)
        camera_info = robot.camera.get_camera_info()
        
        if camera_info:
            print(f"\n4. Camera Info:")
            print(f"   - Resolution: {camera_info['width']}x{camera_info['height']}")
            print(f"   - Distortion model: {camera_info['distortion_model']}")
        
        # Stream for a few seconds
        print("\n5. Streaming 10 frames...")
        time.sleep(2)
        
        # Save one frame
        print(f"\n6. Saving current frame to {args.output}...")
        if robot.save_image(args.output):
            print(f"   ✓ Image saved!")
        else:
            print(f"   ✗ Failed to save image")
        
        # Stop streaming
        print("\n7. Stopping camera stream...")
        robot.stop_camera()
        
        # Stop camera service
        print("\n8. Stopping camera service...")
        robot.system.stop_camera()
        
        print("\n" + "="*60)
        print("✅ Camera test completed successfully!")
        print(f"Total frames received: {frame_count}")
        print("="*60)

if __name__ == '__main__':
    main()
