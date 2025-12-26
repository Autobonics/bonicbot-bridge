#!/usr/bin/env python3
"""
Test script for the move() method
Tests combined linear and angular velocity
"""

import argparse
import time
from bonicbot_bridge import BonicBot

def main():
    parser = argparse.ArgumentParser(description='Test move() method')
    parser.add_argument('--host', type=str, required=True, help='Robot IP address')
    parser.add_argument('--port', type=int, default=9090, help='ROS bridge port')
    args = parser.parse_args()
    
    print("=" * 60)
    print("Testing move() Method - Combined Movements")
    print("=" * 60)
    
    with BonicBot(host=args.host, port=args.port) as robot:
        print("✓ Connected!\n")
        
        # Test 1: Forward only
        print("🔹 Test 1: Pure forward movement")
        print("   move(linear_x=0.3) for 2 seconds")
        start = time.time()
        while (time.time() - start) < 2.0:
            robot.motion.move(linear_x=0.3)
            time.sleep(0.1)
        robot.stop()
        print("   ✓ Complete\n")
        time.sleep(1)
        
        # Test 2: Rotation only
        print("🔹 Test 2: Pure rotation (spin in place)")
        print("   move(angular_z=30) for 2 seconds")
        start = time.time()
        while (time.time() - start) < 2.0:
            robot.motion.move(angular_z=30.0)  # 30 deg/s
            time.sleep(0.1)
        robot.stop()
        print("   ✓ Complete\n")
        time.sleep(1)
        
        # Test 3: Circular arc (forward + rotation)
        print("🔹 Test 3: Circular arc movement")
        print("   move(linear_x=0.2, angular_z=20) for 3 seconds")
        print("   (forward while turning = driving in a circle)")
        start = time.time()
        while (time.time() - start) < 3.0:
            robot.motion.move(linear_x=0.2, angular_z=20.0)
            time.sleep(0.1)
        robot.stop()
        print("   ✓ Complete\n")
        time.sleep(1)
        
        # Test 4: Figure-8 pattern
        print("🔹 Test 4: Figure-8 pattern")
        
        # First arc (turn left)
        print("   First arc: turning left...")
        start = time.time()
        while (time.time() - start) < 2.5:
            robot.motion.move(linear_x=0.2, angular_z=30.0)
            time.sleep(0.1)
        
        # Second arc (turn right)  
        print("   Second arc: turning right...")
        start = time.time()
        while (time.time() - start) < 2.5:
            robot.motion.move(linear_x=0.2, angular_z=-30.0)
            time.sleep(0.1)
        
        robot.stop()
        print("   ✓ Complete\n")
        
        print("=" * 60)
        print("✅ All move() tests completed!")
        print("\nThe move() method allows combining:")
        print("  - linear_x: forward/backward (m/s)")
        print("  - linear_y: left/right (m/s) - for omnidirectional")
        print("  - angular_z: rotation (deg/s)")
        print("=" * 60)

if __name__ == '__main__':
    main()
