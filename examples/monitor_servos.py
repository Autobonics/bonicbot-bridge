#!/usr/bin/env python3
"""
Servo Position Monitor
Displays real-time servo positions (arms, grippers, neck)
"""

import argparse
import time
import os

from bonicbot_bridge import BonicBot

def clear_screen():
    """Clear the terminal screen"""
    os.system('clear' if os.name == 'posix' else 'cls')

def format_angle(angle):
    """Format angle with color indicator"""
    return f"{angle:>7.2f}°"

def print_servo_display(angles):
    """Print a formatted display of servo positions"""
    clear_screen()
    
    print("=" * 70)
    print("🤖 BONICBOT SERVO POSITION MONITOR")
    print("=" * 70)
    print()
    
    # Left Arm
    print("👈 LEFT ARM:")
    print(f"   Shoulder: {format_angle(angles['left_shoulder']):>10}")
    print(f"   Elbow:    {format_angle(angles['left_elbow']):>10}")
    print()
    
    # Right Arm
    print("👉 RIGHT ARM:")
    print(f"   Shoulder: {format_angle(angles['right_shoulder']):>10}")
    print(f"   Elbow:    {format_angle(angles['right_elbow']):>10}")
    print()
    
    # Grippers
    print("🤏 GRIPPERS:")
    print(f"   Left:     {format_angle(angles['left_gripper']):>10}")
    print(f"   Right:    {format_angle(angles['right_gripper']):>10}")
    print()
    
    # Neck
    print("👀 NECK:")
    print(f"   Yaw:      {format_angle(angles['neck_yaw']):>10}")
    print()
    
    print("=" * 70)
    print("Press Ctrl+C to exit")
    print("=" * 70)

def main():
    parser = argparse.ArgumentParser(description='Monitor BonicBot servo positions in real-time')
    parser.add_argument('--host', type=str, required=True, help='Robot IP address')
    parser.add_argument('--port', type=int, default=9090, help='ROS bridge port (default: 9090)')
    parser.add_argument('--rate', type=float, default=0.5, help='Update rate in seconds (default: 0.5)')
    args = parser.parse_args()
    
    print(f"Connecting to robot at {args.host}:{args.port}...")
    
    try:
        with BonicBot(host=args.host, port=args.port) as robot:
            print("✓ Connected! Starting monitor...")
            time.sleep(1)
            
            # Monitor loop
            while True:
                # Get current servo angles
                angles = robot.servo.get_servo_angles()
                
                # Display
                print_servo_display(angles)
                
                # Wait before next update
                time.sleep(args.rate)
                
    except KeyboardInterrupt:
        clear_screen()
        print("\n👋 Servo monitor stopped by user")
        print("Disconnecting...")
    except Exception as e:
        print(f"\n❌ Error: {e}")

if __name__ == '__main__':
    main()
