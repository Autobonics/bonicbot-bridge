#!/usr/bin/env python3
"""
Test script for navigation methods
Tests start_navigation, set_initial_pose, go_to, and wait_for_goal
"""

import argparse
import time
from bonicbot_bridge import BonicBot

def main():
    parser = argparse.ArgumentParser(description='Test navigation methods')
    parser.add_argument('--host', type=str, required=True, help='Robot IP address')
    parser.add_argument('--port', type=int, default=9090, help='ROS bridge port')
    args = parser.parse_args()
    
    print("=" * 60)
    print("Navigation System Test")
    print("=" * 60)
    
    with BonicBot(host=args.host, port=args.port) as robot:
        print("✓ Connected!\n")
        
        # Test 1: Start Navigation System
        print("🔹 Test 1: Starting navigation system")
        try:
            robot.start_navigation()
            print("   ✓ Navigation system started\n")
        except Exception as e:
            print(f"   ❌ Error: {e}\n")
            return
        
        time.sleep(1)
        
        

if __name__ == '__main__':
    main()
