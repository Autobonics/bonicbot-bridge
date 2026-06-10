"""
BonicBot Bridge – CLI diagnostic utility

Entry point for the ``bonicbot-test`` console script defined in setup.py.
Performs a quick connectivity and subsystem health-check against a BonicBot.

Usage:
    bonicbot-test                    # test against localhost:9090
    bonicbot-test --host 192.168.1.5 # test against a remote robot
    bonicbot-test --host bonic.local --port 9090 --timeout 15
"""

import argparse
import sys
import time


def _print_header():
    print()
    print("=" * 52)
    print("  🤖  BonicBot Bridge – Diagnostic Test Utility")
    print("=" * 52)
    print()


def _check(label, fn):
    """Run *fn* and print a PASS / FAIL line."""
    try:
        result = fn()
        print(f"  ✅  {label}: {result}")
        return True
    except Exception as exc:
        print(f"  ❌  {label}: {exc}")
        return False


def main():
    """Entry point invoked by the ``bonicbot-test`` console script."""

    parser = argparse.ArgumentParser(
        description="BonicBot Bridge diagnostic test utility",
    )
    parser.add_argument(
        "--host",
        default="localhost",
        help="Robot IP address or hostname (default: localhost)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9090,
        help="rosbridge port (default: 9090)",
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=10,
        help="Connection timeout in seconds (default: 10)",
    )
    args = parser.parse_args()

    _print_header()

    # --- Connection ---
    print(f"  ⏳  Connecting to {args.host}:{args.port} (timeout {args.timeout}s)...")
    print()

    try:
        from bonicbot_bridge import BonicBot
    except ImportError as exc:
        print(f"  ❌  Import error: {exc}")
        print("      Make sure bonicbot-bridge is installed: pip install -e .")
        sys.exit(1)

    try:
        bot = BonicBot(host=args.host, port=args.port, timeout=args.timeout)
    except Exception as exc:
        print(f"  ❌  Connection FAILED: {exc}")
        sys.exit(1)

    passed = 0
    failed = 0

    # --- Subsystem checks ---
    print("  --- Subsystem checks ---")
    print()

    checks = [
        ("Connection active", lambda: bot.is_connected()),
        ("Position data", lambda: bot.get_position()),
        ("Heading (degrees)", lambda: bot.get_heading()),
        ("Battery level", lambda: bot.get_battery()),
        ("Navigation status", lambda: bot.get_nav_status()),
        ("Servo angles", lambda: bot.servo.get_servo_angles()),
        ("System status", lambda: bot.get_system_status()),
    ]

    # Allow a moment for initial sensor callbacks to arrive
    time.sleep(1.0)

    for label, fn in checks:
        if _check(label, fn):
            passed += 1
        else:
            failed += 1

    # --- Summary ---
    print()
    print("-" * 52)
    total = passed + failed
    print(f"  Results: {passed}/{total} checks passed", end="")
    if failed:
        print(f"  ({failed} failed)")
    else:
        print("  🎉")
    print("-" * 52)
    print()

    # --- Cleanup ---
    try:
        bot.disconnect()
    except Exception:
        pass

    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
