#!/usr/bin/env python3
"""
Servo Test Script
Tests servo control functionality including individual gripper control
"""

import argparse
import time
from bonicbot_bridge import BonicBot

def main():
    parser = argparse.ArgumentParser(description='Test BonicBot servo functionality')
    parser.add_argument('--host', type=str, required=True, help='Robot IP address')
    parser.add_argument('--port', type=int, default=9090, help='ROS bridge port (default: 9090)')
    args = parser.parse_args()
    
    print("="*60)
    print("BonicBot Servo Test")
    print("="*60)
    
    # Connect to robot
    print(f"\n1. Connecting to robot at {args.host}:{args.port}...")
    with BonicBot(host=args.host, port=args.port) as robot:
        print("   ✓ Connected!")
        
        time.sleep(1)
        
        # Reset all servos to neutral
        print("\n2. Resetting all servos to neutral (0°)...")
        robot.reset_servos()
        time.sleep(1)
        
        # Display servo limits
        print("\n3. Servo limits:")
        limits = robot.servo.get_servo_limits()
        for joint, (min_angle, max_angle) in limits.items():
            print(f"   {joint}: [{min_angle}°, {max_angle}°]")
        
        # Test left arm
        print("\n4. Testing left arm...")
        print("   Moving shoulder to 90°, elbow to 30°")
        robot.move_left_arm(90, 30)
        time.sleep(1.5)
        
        print("   Moving shoulder to 45°, elbow to 10°")
        robot.move_left_arm(45, 10)
        time.sleep(1.5)
        
        print("   Returning to neutral")
        robot.move_left_arm(0, 0)
        time.sleep(1)
        
        # Test right arm
        print("\n5. Testing right arm...")
        print("   Moving shoulder to 90°, elbow to 30°")
        robot.move_right_arm(90, 30)
        time.sleep(1.5)
        
        print("   Moving shoulder to 45°, elbow to 10°")
        robot.move_right_arm(45, 10)
        time.sleep(1.5)
        
        print("   Returning to neutral")
        robot.move_right_arm(0, 0)
        time.sleep(1)
        
        # Test both grippers together
        print("\n6. Testing both grippers together...")
        print("   Opening both grippers")
        robot.open_grippers()
        time.sleep(1.5)
        
        print("   Closing both grippers")
        robot.close_grippers()
        time.sleep(1.5)
        
        print("   Returning to neutral")
        robot.set_grippers(0, 0)
        time.sleep(1)
        
        # Test individual grippers
        print("\n7. Testing individual grippers...")
        print("   Opening left gripper only (45°)")
        robot.servo.set_left_gripper(45)
        time.sleep(1.5)
        
        print("   Opening right gripper only (45°)")
        robot.servo.set_right_gripper(45)
        time.sleep(1.5)
        
        print("   Closing left gripper")
        robot.servo.set_left_gripper(0)
        time.sleep(1.5)
        
        print("   Closing right gripper")
        robot.servo.set_right_gripper(0)
        time.sleep(1)
        
        # Test neck
        print("\n8. Testing neck...")
        print("   Looking left")
        robot.look_left()
        time.sleep(1.5)
        
        print("   Looking right")
        robot.look_right()
        time.sleep(1.5)
        
        print("   Looking center")
        robot.look_center()
        time.sleep(1)
        
        # Final reset
        print("\n9. Final reset to neutral...")
        robot.reset_servos()
        time.sleep(1)
        
        # Display current servo angles
        print("\n10. Current servo angles:")
        angles = robot.servo.get_servo_angles()
        for joint, angle in angles.items():
            print(f"   {joint}: {angle:.2f}°")
        
        print("\n" + "="*60)
        print("✅ Servo test completed successfully!")
        print("\nNew features tested:")
        print("  - Separate ROS2 controller topics")
        print("  - Individual gripper control methods")
        print("  - All movements use degrees API")
        print("="*60)

if __name__ == '__main__':
    main()
