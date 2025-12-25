#!/usr/bin/env python3
"""
Integrated Demo Script
Demonstrates combined use of camera, servos, and navigation
"""

import argparse
import time
from bonicbot_bridge import BonicBot

def main():
    parser = argparse.ArgumentParser(description='Integrated BonicBot demo')
    parser.add_argument('--host', type=str, required=True, help='Robot IP address')
    parser.add_argument('--port', type=int, default=9090, help='ROS bridge port (default: 9090)')
    args = parser.parse_args()
    
    print("="*60)
    print("BonicBot Integrated Demo")
    print("="*60)
    
    # Connect to robot
    print(f"\nConnecting to robot at {args.host}:{args.port}...")
    with BonicBot(host=args.host, port=args.port) as robot:
        print("✓ Connected!")
        
        # Check system status
        print("\n📊 System Status:")
        status = robot.system.get_system_status()
        for key, value in status.items():
            print(f"   {key}: {value}")
        
        # Start camera
        print("\n📷 Starting camera...")
        robot.system.start_camera()
        time.sleep(1)
        
        image_count = 0
        def count_images(img):
            nonlocal image_count
            image_count += 1
        
        robot.start_camera(callback=count_images)
        robot.camera.wait_for_image(timeout=3.0)
        print(f"   ✓ Camera streaming (resolution: {robot.camera.get_camera_info()})")
        
        # Wave while capturing images
        print("\n👋 Waving arms while streaming...")
        robot.servo.wave_left_arm(duration=2.0)
        time.sleep(0.5)
        robot.servo.wave_right_arm(duration=2.0)
        
        # Save a snapshot
        print("\n📸 Taking snapshot...")
        robot.save_image("demo_snapshot.jpg")
        
        # Move robot and look around
        print("\n🚗 Moving forward while looking around...")
        robot.look_left()
        robot.move_forward(speed=0.2, duration=1.5)
        
        robot.look_center()
        robot.move_forward(speed=0.2, duration=1.5)
        
        robot.look_right()
        robot.move_forward(speed=0.2, duration=1.5)
        
        robot.look_center()
        robot.stop()
        
        # Gripper demo
        print("\n🤏 Gripper demonstration...")
        print("   Positioning arms")
        robot.move_left_arm(45, 20)
        robot.move_right_arm(45, 20)
        time.sleep(1)
        
        print("   Opening grippers")
        robot.open_grippers()
        time.sleep(1)
        
        print("   Closing grippers")
        robot.close_grippers()
        time.sleep(1)
        
        # Reset
        print("\n🔄 Resetting to neutral position...")
        robot.reset_servos()
        time.sleep(1)
        
        # Final snapshot
        print("\n📸 Final snapshot...")
        robot.save_image("demo_final.jpg")
        
        # Stop camera
        print("\n🛑 Stopping camera...")
        robot.stop_camera()
        robot.system.stop_camera()
        
        # Summary
        print("\n" + "="*60)
        print("✅ Demo completed successfully!")
        print(f"   Images processed: {image_count}")
        print(f"   Snapshots saved: demo_snapshot.jpg, demo_final.jpg")
        print("="*60)

if __name__ == '__main__':
    main()
