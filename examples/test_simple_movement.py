#!/usr/bin/env python3
"""
Simple Movement Testing Script - FIXED VERSION
Tests basic robot movements with proper continuous command publishing
"""

import argparse
import time
from bonicbot_bridge import BonicBot

def main():
    parser = argparse.ArgumentParser(description='Simple movement test for BonicBot')
    parser.add_argument('--host', type=str, required=True, help='Robot IP address')
    parser.add_argument('--port', type=int, default=9090, help='ROS bridge port (default: 9090)')
    args = parser.parse_args()
    
    print("=" * 60)
    print("BonicBot Simple Movement Test (FIXED)")
    print("=" * 60)
    
    # Connect to robot
    print(f"\nConnecting to robot at {args.host}:{args.port}...")
    with BonicBot(host=args.host, port=args.port) as robot:
        print("✓ Connected!\n")
        
        # Test 1: Move Forward for 2 seconds
        print("🔹 Test 1: Moving forward at 0.5 m/s for 2 seconds")
        print("   Expected distance: ~1.0m")
        robot.move_forward(speed=0.5, duration=2.0)
        print("   ✓ Complete\n")
        time.sleep(1)
        
        # Test 2: Move Backward for 5 seconds (should go back further)
        print("🔹 Test 2: Moving backward at 0.5 m/s for 5 seconds")
        print("   Expected distance: ~2.5m backward")
        robot.move_backward(speed=0.5, duration=5.0)
        print("   ✓ Complete")
        print("   → Robot should now be ~1.5m BEHIND start position\n")
        time.sleep(1)
        
        # Test 3: Return forward
        print("🔹 Test 3: Moving forward at 0.5 m/s for 3 seconds")
        print("   Expected distance: ~1.5m forward")
        robot.move_forward(speed=0.5, duration=3.0)
        print("   ✓ Complete")
        print("   → Robot should be back near starting position\n")

        # Test 4: Move Left and Right
        print("🔹 Test 4: Moving left and right at 0.5 m/s for 2 seconds")
        print("   Expected distance: 90degrees left and right")
        robot.turn_left(speed=45, duration=2.0)
        print("   ✓ Complete")
        print("   → Robot should now be 90degrees LEFT of start position\n")
        robot.turn_right(speed=45, duration=2.0)
        print("   ✓ Complete")
        print("   → Robot should now be 90degrees RIGHT of start position\n")
        
        
        print("=" * 60)
        print("✅ All tests completed!")
        print("\nThe duration parameter now works correctly!")
        print("Commands are continuously published at 10Hz to avoid")
        print("ROS2's cmd_vel_timeout (0.5s) auto-stop behavior.")
        print("=" * 60)

if __name__ == '__main__':
    main()
