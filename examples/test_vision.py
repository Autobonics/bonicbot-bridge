"""
Vision Pipeline Test — Camera Stream + Object Detection
========================================================
Connects to the robot via rosbridge, starts the camera,
displays a live OpenCV window with detection overlay, then
enables OBJECT detection mode and prints results in real-time.

Press 'q' in the OpenCV window to quit cleanly.

Fixes vs original
─────────────────
1. bbox format: pipeline publishes normalised [cx, cy, w, h] (0-1).
   Converted to pixel [x1, y1, x2, y2] before drawing.
2. Race condition: wait for yolo_active=True before entering the loop,
   instead of a blind sleep(2).
3. Mode HUD: reads yolo_enabled property (Bool from status topic) so it
   reflects reality even before the first detection arrives.
4. Detection terminal print is frame-rate-throttled (1 Hz) not tied to
   frame_count==1, which was always True on the first frame.
"""

import argparse
import time
import cv2
import numpy as np
from bonicbot_bridge import BonicBot

# ── Configuration ────────────────────────────────────────────────────────────
WINDOW_NAME = "BonicBot Vision — Object Detection"

# How long to wait for the vision pipeline to start on the RPi (model loading)
VISION_READY_TIMEOUT = 30.0


# ── Drawing helper ───────────────────────────────────────────────────────────

def draw_detections(frame: np.ndarray, detections: list) -> np.ndarray:
    """
    Draw bounding boxes and labels on *frame*.

    The pipeline publishes bbox as [cx_norm, cy_norm, w_norm, h_norm] — all
    values normalised 0→1 relative to the model input size — so we convert to
    pixel [x1, y1, x2, y2] using the actual frame dimensions here.
    """
    h_px, w_px = frame.shape[:2]

    for det in detections:
        cls_name = det.get("class", "?")
        conf     = det.get("confidence", 0.0)
        bbox     = det.get("bbox")

        if not bbox or len(bbox) < 4:
            continue

        cx_n, cy_n, bw_n, bh_n = bbox

        # Convert normalised centre-wh → pixel top-left / bottom-right
        x1 = int((cx_n - bw_n / 2) * w_px)
        y1 = int((cy_n - bh_n / 2) * h_px)
        x2 = int((cx_n + bw_n / 2) * w_px)
        y2 = int((cy_n + bh_n / 2) * h_px)

        # Clamp to frame bounds
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w_px - 1, x2), min(h_px - 1, y2)

        color = (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        label = f"{cls_name} {conf:.0%}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        cv2.rectangle(frame, (x1, y1 - th - 8), (x1 + tw + 4, y1), color, -1)
        cv2.putText(
            frame, label, (x1 + 2, y1 - 4),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA,
        )

    return frame


# ── Wait helper ──────────────────────────────────────────────────────────────

def wait_for_vision_ready(bot: "BonicBot", timeout: float = VISION_READY_TIMEOUT) -> bool:
    """
    Block until the robot confirms yolo_active=True via the status topic,
    replacing the original blind sleep(2) that was too short on RPi 4.
    """
    print(f"⏳ Waiting for YOLO active confirmation (up to {timeout:.0f}s)…")
    deadline = time.time() + timeout
    while time.time() < deadline:
        if bot.vision.yolo_enabled:
            return True
        time.sleep(0.1)
    return False

# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Test BonicBot vision pipeline')
    parser.add_argument('--host', type=str, required=True, help='Robot IP address')
    parser.add_argument('--port', type=int, default=9090, help='ROS bridge port (default: 9090)')
    args = parser.parse_args()

    print(f"🔌 Connecting to robot at {args.host}:{args.port}...")
    bot = BonicBot(host=args.host, port=args.port)

    try:
        # ── Step 1: Camera hardware + streaming ───────────────────────────────
        print("\n📷 Starting camera service and streaming...")
        bot.system.start_camera()
        bot.start_camera()

        print("⏳ Waiting for first frame...")
        if not bot.camera.wait_for_image(timeout=5.0):
            print("❌ No image received within 5 s — check camera hardware!")
            return
        print("✅ Camera stream is live!\n")

        # ── Step 2: Vision pipeline + YOLO ───────────────────────────────────
        print("🚀 Starting vision pipeline on robot...")
        bot.vision.start_vision()

        print("👁️  Enabling YOLO detection mode...")
        bot.vision.enable_detection("yolo")

        # Wait for the RPi to confirm the detector is actually running.
        # On a Pi 4 ONNX model load can take 4-8 s — polling the Bool status
        # topic is reliable; a hard sleep is not.
        if not wait_for_vision_ready(bot):
            print(
                "⚠️  YOLO did not confirm active within timeout.\n"
                "   Check the robot logs: ros2 run bonicbot vision_pipeline\n"
                "   Possible causes: missing ~/models/yolov8n.onnx, "
                "onnxruntime not installed, camera not started."
            )
            # Continue anyway — we might still get detections eventually
        else:
            print(f"✅ YOLO active confirmed by robot!\n")

        # ── Step 3: Live display loop ─────────────────────────────────────────
        print("=" * 52)
        print("  LIVE STREAM + OBJECT DETECTION")
        print("  Press 'q' in the OpenCV window to quit")
        print("=" * 52)

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 800, 600)

        fps_start      = time.time()
        frame_count    = 0
        fps            = 0.0
        last_print_t   = 0.0      # for 1 Hz terminal print throttle

        while True:
            frame = bot.get_image()

            if frame is not None:
                detections = bot.vision.get_detections()
                display    = draw_detections(frame.copy(), detections)

                # FPS counter
                frame_count += 1
                elapsed = time.time() - fps_start
                if elapsed >= 1.0:
                    fps         = frame_count / elapsed
                    frame_count = 0
                    fps_start   = time.time()

                # Determine mode label from Bool status topic (always accurate)
                if bot.vision.yolo_enabled:
                    mode_label = "yolo"
                elif bot.vision.face_enabled:
                    mode_label = "face"
                elif bot.vision.pose_enabled:
                    mode_label = "pose"
                elif bot.vision.gesture_enabled:
                    mode_label = "gesture"
                elif bot.vision.aruco_enabled:
                    mode_label = "aruco"
                else:
                    mode_label = "none"

                # HUD overlay
                cv2.putText(display, f"FPS: {fps:.1f}",
                            (10, 30),  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(display, f"Mode: {mode_label}",
                            (10, 60),  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(display, f"Detections: {len(detections)}",
                            (10, 90),  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                cv2.imshow(WINDOW_NAME, display)

                # Terminal print — throttled to once per second
                now = time.time()
                if detections and (now - last_print_t) >= 1.0:
                    last_print_t = now
                    for d in detections:
                        print(
                            f"  🎯 {d.get('class', '?'):15s} "
                            f"conf={d.get('confidence', 0):.2f}  "
                            f"bbox={[round(v, 3) for v in d.get('bbox', [])]}"
                        )

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("\n👋 Quit requested.")
                break

            time.sleep(0.03)   # ~30 Hz poll

    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user (Ctrl+C)")

    finally:
        print("\n🧹 Cleaning up...")
        cv2.destroyAllWindows()

        for cleanup in (
            bot.vision.disable_detection,
            bot.stop_camera,
            bot.system.stop_camera,
        ):
            try:
                cleanup()
            except Exception:
                pass

        bot.disconnect()
        print("✅ Done.")


if __name__ == "__main__":
    main()