#!/usr/bin/env python3
"""
Position Monitor
Displays real-time robot position (x, y, theta) from odometry/TF
"""

import argparse
import time
import os
import math

from bonicbot_bridge import BonicBot

def clear_screen():
    """Clear the terminal screen"""
    os.system('clear' if os.name == 'posix' else 'cls')

def format_position(pos):
    """Format position data for display"""
    if pos is None:
        return "No position data available"
    
    x = pos.get('x', 0.0)
    y = pos.get('y', 0.0)
    theta = pos.get('theta', 0.0)
    theta_deg = math.degrees(theta)
    
    return f"X: {x:>7.3f}m  |  Y: {y:>7.3f}m  |  θ: {theta:>6.3f} rad ({theta_deg:>7.2f}°)"

def print_position_display(bot, start_pos=None):
    """Print formatted position display"""
    clear_screen()
    
    print("=" * 80)
    print("🤖 BONICBOT POSITION MONITOR")
    print("=" * 80)
    print()
    
    # Current position
    pos = bot.get_position()
    
    if pos:
        print("📍 CURRENT POSITION:")
        print(f"   {format_position(pos)}")
        print()
        
        # Individual components
        print("📊 COMPONENTS:")
        print(f"   X:     {pos['x']:>10.4f} m")
        print(f"   Y:     {pos['y']:>10.4f} m")
        print(f"   Theta: {pos['theta']:>10.4f} rad ({math.degrees(pos['theta']):>8.2f}°)")
        print()
        
        # Distance from start
        if start_pos:
            dx = pos['x'] - start_pos['x']
            dy = pos['y'] - start_pos['y']
            distance = math.sqrt(dx*dx + dy*dy)
            
            print("📏 FROM START POSITION:")
            print(f"   ΔX:       {dx:>10.4f} m")
            print(f"   ΔY:       {dy:>10.4f} m")
            print(f"   Distance: {distance:>10.4f} m")
            print()
        
        # Velocity (if available)
        velocity = bot.sensors.motion.get_current_velocity() if hasattr(bot.sensors, 'motion') else bot.motion.get_current_velocity() if hasattr(bot.motion, 'get_current_velocity') else None
        
        if velocity is None:
            # Try using the sensors getter
            try:
                twist = bot.sensors.current_pose
                if twist and 'twist' in str(twist):
                    # This is odometry data
                    pass
            except:
                pass
        
        print("🚗 STATUS:")
        is_moving = bot.motion.is_moving()
        print(f"   Moving: {'Yes ✓' if is_moving else 'No'}")
        
    else:
        print("⚠️  Waiting for position data...")
    
    print()
    print("=" * 80)
    print("Press Ctrl+C to exit")
    print("=" * 80)

def main():
    parser = argparse.ArgumentParser(description='Monitor BonicBot position in real-time')
    parser.add_argument('--host', type=str, required=True, help='Robot IP address')
    parser.add_argument('--port', type=int, default=9090, help='ROS bridge port (default: 9090)')
    parser.add_argument('--rate', type=float, default=0.5, help='Update rate in seconds (default: 0.5)')
    parser.add_argument('--save-start', action='store_true', help='Save starting position for distance tracking')
    args = parser.parse_args()
    
    print(f"Connecting to robot at {args.host}:{args.port}...")
    
    try:
        with BonicBot(host=args.host, port=args.port) as robot:
            print("✓ Connected! Waiting for position data...")
            
            # Wait for first position
            time.sleep(1.5)
            
            # Save start position if requested
            start_pos = None
            if args.save_start:
                start_pos = robot.get_position()
                if start_pos:
                    print(f"📌 Start position saved: ({start_pos['x']:.3f}, {start_pos['y']:.3f})")
                    time.sleep(1)
            
            # Monitor loop
            while True:
                print_position_display(robot, start_pos)
                time.sleep(args.rate)
                
    except KeyboardInterrupt:
        clear_screen()
        print("\n👋 Position monitor stopped by user")
        print("Disconnecting...")
    except Exception as e:
        print(f"\n❌ Error: {e}")

if __name__ == '__main__':
    main()
