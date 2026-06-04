"""
=============================================================================
  Ball Balancing Platform — with Motor Command Visualizer
=============================================================================

FIXES / NEW IN THIS VERSION
-----------------------------
  1. Serial port is now configurable from the GUI (no need to edit code)
  2. Motor command visualizer shows live X/Y tilt as animated arrows + bars
  3. Debug panel shows exactly what command is being sent each frame
  4. "Test Motors" button sends a sweep to confirm servos are responding
  5. Platform Hough-circle detection (robust, no colour marker needed)
  6. Ball mask restricted to platform interior only

KEYBOARD SHORTCUTS
-------------------
    Space  →  Start / Stop
    R      →  Re-detect platform
    T      →  Test motors (sends sweep while stopped)
    ESC    →  Quit
=============================================================================
"""

import csv
import math
import time
import logging
import threading
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
from PIL import Image, ImageTk
import tkinter as tk
import tkinter.font as tkfont

try:
    import serial
    import serial.tools.list_ports
    SERIAL_OK = True
except ImportError:
    SERIAL_OK = False
    logging.warning("pyserial not installed — serial disabled.")

# =============================================================================
#  CONFIG
# =============================================================================

CONFIG = {
    "camera_index":    3,
    "camera_url":      "",
    "droidcam_port":   4747,
    "frame_width":     640,
    "frame_height":    480,
    "target_fps":      30,
    "warmup_frames":   20,

    # Hough circle platform detection
    "hough_dp":              1.2,
    "hough_min_dist":        200,
    "hough_param1":          60,
    "hough_param2":          30,
    "hough_min_radius_frac": 0.20,
    "hough_max_radius_frac": 0.55,
    "platform_lock_frames":  40,

    # Ball HSV (orange)
    "hsv_lower":  [5,  120, 120],
    "hsv_upper":  [25, 255, 255],

    # Ball detection
    "min_ball_area":  150,
    "ema_alpha":      0.35,
    "morph_open_k":   5,
    "morph_close_k":  9,

    # PID
    "pid_kp":          0.35,
    "pid_ki":          0.005,
    "pid_kd":          0.12,
    "pid_output_limit":   50.0,
    "pid_integral_limit": 40.0,
    "pid_kp_step":     0.01,
    "pid_ki_step":     0.001,
    "pid_kd_step":     0.01,

    # Serial
    "serial_port":     "COM5",
    "serial_baud":     115200,
    "serial_timeout":  0.05,
    "serial_send_every":         1,   # send EVERY frame for responsive motors
    "serial_reconnect_interval": 2.0,

    # Safety
    "max_lost_frames": 20,

    # Visuals
    "trajectory_len": 60,
    "tolerance_px":   25,
}

# =============================================================================
#  SHARED STATE
# =============================================================================

@dataclass
class AppState:
    lock: threading.Lock = field(default_factory=threading.Lock)

    running:   bool = False
    quit_flag: bool = False

    reconnect_camera:  bool = False
    redetect_platform: bool = False
    test_motors_flag:  bool = False   # GUI sets True to run motor test
    camera_url:        str  = ""

    ball_detected:    bool  = False
    ball_x:           float = 0.0
    ball_y:           float = 0.0
    ball_lost_frames: int   = 0

    centre_x:      float = 0.0
    centre_y:      float = 0.0
    platform_r:    float = 0.0
    centre_locked: bool  = False

    error_x:  float = 0.0
    error_y:  float = 0.0
    output_x: float = 0.0
    output_y: float = 0.0
    distance: float = 0.0

    pid_kp: float = CONFIG["pid_kp"]
    pid_ki: float = CONFIG["pid_ki"]
    pid_kd: float = CONFIG["pid_kd"]

    hsv_lower: list = field(default_factory=lambda: list(CONFIG["hsv_lower"]))
    hsv_upper: list = field(default_factory=lambda: list(CONFIG["hsv_upper"]))

    fps:              float = 0.0
    loop_ms:          float = 0.0
    serial_connected: bool  = False
    serial_port_used: str   = CONFIG["serial_port"]
    last_command:     str   = "—"
    ack_status:       str   = "—"
    commands_sent:    int   = 0       # running counter
    frame_count:      int   = 0
    camera_connected: bool  = False
    camera_source:    str   = "—"

    display_frame: Optional[np.ndarray] = None
    mask_frame:    Optional[np.ndarray] = None
    raw_frame:     Optional[np.ndarray] = None


# =============================================================================
#  CAMERA MANAGER
# =============================================================================

class CameraManager:
    def __init__(self):
        self._cap = None
        self.actual_w = CONFIG["frame_width"]
        self.actual_h = CONFIG["frame_height"]
        self.source = ""

    def open(self, source=None) -> bool:
        self.release()
        if source is None:
            url = CONFIG.get("camera_url", "").strip()
            source = url if url else CONFIG["camera_index"]
        if isinstance(source, str) and source:
            source = self._normalise_url(source)
        self.source = str(source)
        self._cap = cv2.VideoCapture(source)
        if not self._cap.isOpened():
            logging.error("Cannot open camera: %s", source)
            return False
        if isinstance(source, int):
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CONFIG["frame_width"])
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CONFIG["frame_height"])
            self._cap.set(cv2.CAP_PROP_FPS,          CONFIG["target_fps"])
        self.actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return True

    @staticmethod
    def _normalise_url(raw: str) -> str:
        raw = raw.strip()
        port = CONFIG["droidcam_port"]
        if raw.startswith("http") and "/video" in raw:
            return raw
        if raw.startswith("http://") or raw.startswith("rtsp://"):
            return raw.rstrip("/") + "/video"
        if ":" in raw:
            return f"http://{raw}/video"
        return f"http://{raw}:{port}/video"

    def grab(self) -> Tuple[bool, Optional[np.ndarray]]:
        if not self._cap or not self._cap.isOpened():
            return False, None
        ok, frame = self._cap.read()
        return (ok, frame) if (ok and frame is not None) else (False, None)

    def release(self):
        if self._cap:
            self._cap.release()
            self._cap = None


# =============================================================================
#  PLATFORM DETECTOR  (Hough Circle)
# =============================================================================

class PlatformDetector:
    BUCKET = 10

    def __init__(self, w: int, h: int):
        self._w = w; self._h = h
        self._locked = False
        self._cx = w // 2; self._cy = h // 2
        self._r  = int(min(w, h) * 0.38)
        self._frames = 0
        self._votes: dict = {}

    @property
    def centre(self): return (self._cx, self._cy)
    @property
    def radius(self): return self._r
    @property
    def locked(self): return self._locked

    def reset(self):
        self._locked = False; self._frames = 0; self._votes = {}

    def detect(self, frame, state) -> Tuple[int, int, int]:
        if self._locked:
            return self._cx, self._cy, self._r

        circles = self._find_circles(frame)
        if circles:
            for (x, y, r) in circles:
                key = (round(x / self.BUCKET) * self.BUCKET,
                       round(y / self.BUCKET) * self.BUCKET)
                if key in self._votes:
                    old = self._votes[key]; n = old[3] + 1
                    self._votes[key] = (
                        (old[0]*old[3]+x)/n, (old[1]*old[3]+y)/n,
                        (old[2]*old[3]+r)/n, n)
                else:
                    self._votes[key] = (x, y, r, 1)

        self._frames += 1
        if self._frames >= CONFIG["platform_lock_frames"]:
            self._lock_best()
            with state.lock:
                state.centre_locked = True
                state.centre_x = self._cx
                state.centre_y = self._cy
                state.platform_r = self._r

        if self._votes:
            best = max(self._votes.values(), key=lambda v: v[3])
            return int(best[0]), int(best[1]), int(best[2])
        return self._cx, self._cy, self._r

    def _lock_best(self):
        if not self._votes:
            self._locked = True; return
        best = max(self._votes.values(), key=lambda v: v[3])
        self._cx = int(best[0]); self._cy = int(best[1]); self._r = int(best[2])
        self._locked = True
        logging.info("Platform LOCKED → (%d,%d) r=%d votes=%d",
                     self._cx, self._cy, self._r, int(best[3]))

    def _find_circles(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.bilateralFilter(gray, 9, 75, 75)
        min_dim = min(self._w, self._h)
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT,
            dp=CONFIG["hough_dp"], minDist=CONFIG["hough_min_dist"],
            param1=CONFIG["hough_param1"], param2=CONFIG["hough_param2"],
            minRadius=int(min_dim * CONFIG["hough_min_radius_frac"]),
            maxRadius=int(min_dim * CONFIG["hough_max_radius_frac"]))
        if circles is None: return None
        circles = np.round(circles[0]).astype(int)
        mx, my = self._w * 0.20, self._h * 0.20
        valid = [(x,y,r) for x,y,r in circles
                 if mx < x < self._w-mx and my < y < self._h-my]
        return valid or None


# =============================================================================
#  BALL DETECTOR
# =============================================================================

class BallDetector:
    def __init__(self, w: int, h: int):
        ok = CONFIG["morph_open_k"]; ck = CONFIG["morph_close_k"]
        self._open_k  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ok, ok))
        self._close_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ck, ck))
        self._w = w; self._h = h
        self._platform_mask = np.zeros((h, w), dtype=np.uint8)
        self._update_mask(w//2, h//2, int(min(w,h)*0.38))
        self._smooth_x: Optional[float] = None
        self._smooth_y: Optional[float] = None

    def update_platform_mask(self, cx, cy, r):
        self._update_mask(cx, cy, r)

    def _update_mask(self, cx, cy, r):
        self._platform_mask = np.zeros((self._h, self._w), dtype=np.uint8)
        cv2.circle(self._platform_mask, (cx, cy), max(10, r-8), 255, -1)

    def detect(self, frame, state):
        alpha = CONFIG["ema_alpha"]
        with state.lock:
            lower = np.array(state.hsv_lower, dtype=np.uint8)
            upper = np.array(state.hsv_upper, dtype=np.uint8)
        blurred = cv2.GaussianBlur(frame, (7, 7), 0)
        hsv  = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._open_k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._close_k)
        mask = cv2.bitwise_and(mask, self._platform_mask)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts: return None, None, None, mask
        largest = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(largest) < CONFIG["min_ball_area"]: return None, None, None, mask
        M = cv2.moments(largest)
        if M["m00"] == 0: return None, None, None, mask
        raw_x = M["m10"] / M["m00"]; raw_y = M["m01"] / M["m00"]
        _, radius = cv2.minEnclosingCircle(largest)
        if self._smooth_x is None:
            self._smooth_x, self._smooth_y = raw_x, raw_y
        else:
            self._smooth_x = alpha * raw_x + (1-alpha) * self._smooth_x
            self._smooth_y = alpha * raw_y + (1-alpha) * self._smooth_y
        return self._smooth_x, self._smooth_y, float(radius), mask

    def reset_smooth(self):
        self._smooth_x = None; self._smooth_y = None


# =============================================================================
#  PID CONTROLLER
# =============================================================================

class PIDController:
    def __init__(self): self.reset()

    def compute(self, error, kp, ki, kd):
        now = time.monotonic()
        dt  = max((now - self._prev_time) if self._prev_time else 0.033, 1e-6)
        self._prev_time = now
        p = kp * error
        self._integral += error * dt
        lim = CONFIG["pid_integral_limit"]
        self._integral = float(np.clip(self._integral, -lim, lim))
        i = ki * self._integral
        d = kd * (error - self._prev_error) / dt
        self._prev_error = error
        return float(np.clip(p+i+d, -CONFIG["pid_output_limit"], CONFIG["pid_output_limit"]))

    def reset(self):
        self._integral = 0.0; self._prev_error = 0.0; self._prev_time = None


# =============================================================================
#  SERIAL MANAGER
# =============================================================================

class SerialManager:
    def __init__(self):
        self._ser  = None
        self._lock = threading.Lock()
        self._last_reconnect = 0.0
        self.connected = False
        self._port = CONFIG["serial_port"]

    def set_port(self, port: str):
        self._port = port

    def try_connect(self) -> bool:
        if not SERIAL_OK: return False
        now = time.monotonic()
        if now - self._last_reconnect < CONFIG["serial_reconnect_interval"]:
            return False
        self._last_reconnect = now
        try:
            self._ser = serial.Serial(self._port, CONFIG["serial_baud"],
                                      timeout=CONFIG["serial_timeout"])
            time.sleep(1.5)
            self.connected = True
            logging.info("Serial connected on %s", self._port)
            return True
        except Exception as e:
            logging.warning("Serial connect failed (%s): %s", self._port, e)
            self.connected = False
            return False

    def send(self, x: float, y: float) -> str:
        if not self.connected or self._ser is None:
            return "DISCONNECTED"
        msg = f"X:{int(x)},Y:{int(y)}\n"
        with self._lock:
            try:
                self._ser.write(msg.encode())
                reply = self._ser.readline().decode(errors="ignore").strip()
                return reply if reply else "SENT"
            except Exception as e:
                logging.warning("Serial send error: %s", e)
                self.connected = False; self._ser = None
                return "ERROR"

    def send_safe(self): self.send(0.0, 0.0)

    def close(self):
        if self._ser:
            try: self._ser.close()
            except: pass
        self.connected = False


# =============================================================================
#  DATA LOGGER
# =============================================================================

class DataLogger:
    COLUMNS = ["timestamp","ball_x","ball_y","centre_x","centre_y",
               "error_x","error_y","distance","output_x","output_y","kp","ki","kd"]

    def __init__(self):
        self._save_dir   = Path.home() / "Documents" / "data"
        self._rows       = []
        self._start_time = time.time()

    def record(self, state: AppState):
        with state.lock:
            if not state.ball_detected: return
            self._rows.append({
                "timestamp": f"{time.time()-self._start_time:.3f}",
                "ball_x":  f"{state.ball_x:.2f}", "ball_y":  f"{state.ball_y:.2f}",
                "centre_x":f"{state.centre_x:.2f}","centre_y":f"{state.centre_y:.2f}",
                "error_x": f"{state.error_x:.2f}", "error_y": f"{state.error_y:.2f}",
                "distance":f"{state.distance:.2f}",
                "output_x":f"{state.output_x:.2f}","output_y":f"{state.output_y:.2f}",
                "kp":f"{state.pid_kp:.4f}","ki":f"{state.pid_ki:.5f}","kd":f"{state.pid_kd:.4f}",
            })

    def save(self):
        if not self._rows: return
        self._save_dir.mkdir(parents=True, exist_ok=True)
        fname = self._save_dir / f"session_{int(time.time())}.csv"
        with open(fname, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=self.COLUMNS)
            w.writeheader(); w.writerows(self._rows)
        print(f"\n✓ Data saved → {fname}  ({len(self._rows)} rows)\n")


# =============================================================================
#  VISION THREAD
# =============================================================================

class VisionThread(threading.Thread):
    def __init__(self, state: AppState):
        super().__init__(daemon=True)
        self._state    = state
        self._camera   = CameraManager()
        self._platform = None
        self._detector = None
        self._pid_x    = PIDController()
        self._pid_y    = PIDController()
        self._serial   = SerialManager()
        self._logger   = DataLogger()
        self._traj     = deque(maxlen=CONFIG["trajectory_len"])
        self._frame_count = 0
        self._fps_timer   = time.monotonic()
        self._fps_frames  = 0
        self._ball_radius = 15.0

    def run(self):
        with self._state.lock:
            initial_url  = self._state.camera_url.strip()
            initial_port = self._state.serial_port_used
        self._serial.set_port(initial_port)

        opened = self._camera.open(initial_url if initial_url else None)
        self._update_camera_status(opened)
        if opened: self._init_detectors()
        self._serial.try_connect()

        while not self._state.quit_flag:
            loop_start = time.monotonic()

            # Handle camera reconnect
            with self._state.lock:
                do_reconnect = self._state.reconnect_camera
                new_url      = self._state.camera_url
                new_port     = self._state.serial_port_used
                if do_reconnect: self._state.reconnect_camera = False
            if do_reconnect:
                self._serial.set_port(new_port)
                opened = self._camera.open(new_url if new_url.strip() else None)
                self._update_camera_status(opened)
                if opened:
                    self._init_detectors()
                    self._frame_count = 0
                    self._pid_x.reset(); self._pid_y.reset()
                # Reconnect serial with potentially new port
                self._serial.close()
                self._serial._last_reconnect = 0
                self._serial.try_connect()
                continue

            # Handle platform re-detect
            with self._state.lock:
                do_redetect = self._state.redetect_platform
                if do_redetect: self._state.redetect_platform = False; self._state.centre_locked = False
            if do_redetect and self._platform:
                self._platform.reset()
                if self._detector:
                    W, H = self._camera.actual_w, self._camera.actual_h
                    self._detector.update_platform_mask(W//2, H//2, int(min(W,H)*0.45))

            # Handle motor test
            with self._state.lock:
                do_test = self._state.test_motors_flag
                if do_test: self._state.test_motors_flag = False
            if do_test and not self._state.running:
                self._run_motor_test()

            if not self._camera._cap or not self._camera._cap.isOpened():
                time.sleep(0.2); continue

            ok, frame = self._camera.grab()
            if not ok:
                time.sleep(0.05); continue

            self._frame_count += 1
            self._fps_frames  += 1

            now = time.monotonic()
            if self._fps_frames >= 30:
                elapsed = now - self._fps_timer
                fps = self._fps_frames / elapsed if elapsed > 0 else 0
                with self._state.lock: self._state.fps = fps
                self._fps_frames = 0; self._fps_timer = now

            with self._state.lock: running = self._state.running

            # Platform detection
            if self._platform and self._detector:
                cx, cy, pr = self._platform.detect(frame, self._state)
                self._detector.update_platform_mask(cx, cy, pr)
                with self._state.lock:
                    self._state.centre_x = cx; self._state.centre_y = cy
                    self._state.platform_r = pr
            else:
                cx = cy = 0; pr = 100

            # Ball detection
            if self._detector:
                ball_x, ball_y, radius, mask = self._detector.detect(frame, self._state)
                if radius: self._ball_radius = radius
            else:
                ball_x = ball_y = radius = None
                mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)

            # Serial keep-alive
            if not self._serial.connected:
                connected = self._serial.try_connect()
                with self._state.lock: self._state.serial_connected = connected
            else:
                with self._state.lock: self._state.serial_connected = True

            warmup_done = self._frame_count > CONFIG["warmup_frames"]

            if running and warmup_done:
                self._run_control(ball_x, ball_y, cx, cy)
                self._logger.record(self._state)
            else:
                with self._state.lock:
                    if ball_x is not None:
                        self._state.ball_x = ball_x; self._state.ball_y = ball_y
                        self._state.ball_detected = True; self._state.ball_lost_frames = 0
                    else:
                        self._state.ball_detected = False; self._state.ball_lost_frames += 1

            display = self._draw_overlay(frame.copy(), cx, cy, pr)
            with self._state.lock:
                self._state.raw_frame     = frame.copy()
                self._state.display_frame = display
                self._state.mask_frame    = mask
                self._state.frame_count   = self._frame_count
                self._state.loop_ms       = (time.monotonic() - loop_start) * 1000

        self._serial.send_safe(); self._serial.close()
        self._camera.release(); self._logger.save()

    def _init_detectors(self):
        W = self._camera.actual_w; H = self._camera.actual_h
        self._platform = PlatformDetector(W, H)
        self._detector = BallDetector(W, H)
        with self._state.lock:
            self._state.centre_x = W//2; self._state.centre_y = H//2
            self._state.centre_locked = False

    def _update_camera_status(self, opened):
        with self._state.lock:
            self._state.camera_connected = opened
            self._state.camera_source    = self._camera.source if opened else "—"

    def _run_motor_test(self):
        """Sweep X and Y to verify servos are responding."""
        logging.info("Motor test: starting sweep")
        steps = [0, 20, 40, 20, 0, -20, -40, -20, 0]
        for v in steps:
            ack = self._serial.send(float(v), 0.0)
            with self._state.lock:
                self._state.last_command = f"X:{v},Y:0  [TEST]"
                self._state.ack_status   = ack
                self._state.output_x = float(v); self._state.output_y = 0.0
            time.sleep(0.18)
        for v in steps:
            ack = self._serial.send(0.0, float(v))
            with self._state.lock:
                self._state.last_command = f"X:0,Y:{v}  [TEST]"
                self._state.ack_status   = ack
                self._state.output_x = 0.0; self._state.output_y = float(v)
            time.sleep(0.18)
        self._serial.send_safe()
        with self._state.lock:
            self._state.last_command = "X:0,Y:0"; self._state.ack_status = "TEST DONE"
            self._state.output_x = 0.0; self._state.output_y = 0.0
        logging.info("Motor test: complete")

    def _run_control(self, ball_x, ball_y, cx, cy):
        state = self._state
        with state.lock:
            kp = state.pid_kp; ki = state.pid_ki; kd = state.pid_kd

        if ball_x is not None:
            self._traj.append((int(ball_x), int(ball_y)))
            error_x = cx - ball_x; error_y = cy - ball_y
            dist    = math.sqrt(error_x**2 + error_y**2)
            out_x   = self._pid_x.compute(error_x, kp, ki, kd)
            out_y   = self._pid_y.compute(error_y, kp, ki, kd)
            with state.lock:
                state.ball_x = ball_x; state.ball_y = ball_y
                state.ball_detected = True; state.ball_lost_frames = 0
                state.error_x = error_x; state.error_y = error_y
                state.output_x = out_x;  state.output_y = out_y
                state.distance = dist
            if self._frame_count % CONFIG["serial_send_every"] == 0:
                ack = self._serial.send(out_x, out_y)
                with state.lock:
                    state.last_command  = f"X:{int(out_x)},Y:{int(out_y)}"
                    state.ack_status    = ack
                    state.commands_sent += 1
        else:
            with state.lock:
                state.ball_detected = False; state.ball_lost_frames += 1
                lost = state.ball_lost_frames
            if lost >= CONFIG["max_lost_frames"]:
                self._pid_x.reset(); self._pid_y.reset()
                self._detector.reset_smooth()
                self._serial.send_safe()
                with state.lock:
                    state.last_command = "X:0,Y:0"; state.ack_status = "SAFE"
                    state.output_x = 0.0; state.output_y = 0.0

    def _draw_overlay(self, frame, cx, cy, pr):
        state = self._state
        with state.lock:
            bx = int(state.ball_x); by = int(state.ball_y)
            detected = state.ball_detected; dist = state.distance
            locked = state.centre_locked; kp = state.pid_kp
            ki = state.pid_ki; kd = state.pid_kd

        tol = CONFIG["tolerance_px"]
        if locked:
            cv2.circle(frame, (cx, cy), pr, (0, 200, 180), 2)
        else:
            for ang in range(0, 360, 20):
                a1 = math.radians(ang); a2 = math.radians(ang+12)
                p1 = (int(cx+pr*math.cos(a1)), int(cy+pr*math.sin(a1)))
                p2 = (int(cx+pr*math.cos(a2)), int(cy+pr*math.sin(a2)))
                cv2.line(frame, p1, p2, (100,200,255), 2)

        color_tol = (0,200,60) if dist < tol else (0,60,220)
        cv2.circle(frame, (cx,cy), tol, color_tol, 1)

        pts = list(self._traj)
        for i in range(1, len(pts)):
            a = i / max(len(pts), 1)
            cv2.line(frame, pts[i-1], pts[i], (0,int(140*a),int(255*a)), 2)

        centre_col = (0,230,230) if locked else (100,100,255)
        cv2.circle(frame, (cx,cy), 8, centre_col, -1)
        cv2.drawMarker(frame, (cx,cy), (255,255,255), cv2.MARKER_CROSS, 28, 2)
        cv2.putText(frame, "LOCKED" if locked else "detecting...",
                    (cx+12, cy-12), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0,230,230) if locked else (80,160,255), 1, cv2.LINE_AA)

        if detected:
            ratio = min(dist/150.0, 1.0)
            cv2.line(frame, (cx,cy), (bx,by),
                     (0,int(255*(1-ratio)),int(255*ratio)), 2)
            cv2.circle(frame, (bx,by), max(12,int(self._ball_radius)), (0,140,255), 2)
            cv2.circle(frame, (bx,by), 3, (0,140,255), -1)

        if self._frame_count <= CONFIG["warmup_frames"]:
            cv2.putText(frame, f"Warming up... {CONFIG['warmup_frames']-self._frame_count}",
                        (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,220,220), 2, cv2.LINE_AA)

        cv2.putText(frame, f"Kp={kp:.3f}  Ki={ki:.4f}  Kd={kd:.3f}",
                    (10, frame.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255,255,180), 1, cv2.LINE_AA)
        return frame


# =============================================================================
#  MOTOR VISUALIZER CANVAS
# =============================================================================

class MotorVisualizer(tk.Canvas):
    """
    A canvas widget that draws a live 2-axis motor command display.

    Layout:
      ┌──────────────────────────────────┐
      │        ↑  Y tilt bar             │
      │   ←────┼────→   X tilt bar       │
      │        ↓                         │
      │  X: +23  Y: -17  CMD: X:23,Y:-17 │
      └──────────────────────────────────┘
    """

    W = 280
    H = 200

    BG_COL   = "#0d1117"
    GRID_COL = "#1e2530"
    ZERO_COL = "#2a3444"
    POS_COL  = "#00e5cc"   # teal  → positive tilt
    NEG_COL  = "#ff4d6d"   # red   → negative tilt
    BALL_COL = "#ffaa00"
    TEXT_COL = "#e2e8f0"
    DIM_COL  = "#455060"

    MAX_VAL  = CONFIG["pid_output_limit"]   # 50

    def __init__(self, parent, **kwargs):
        kwargs.setdefault("bg", self.BG_COL)
        super().__init__(parent, width=self.W, height=self.H,
                         highlightthickness=0, **kwargs)
        self._ox = self.W // 2   # origin X
        self._oy = self.H // 2 - 10  # origin Y (slightly above centre to leave room for labels)
        self._arm = 80           # half-length of the axis arms
        self._bar_w = 12         # width of tilt bars

        self._out_x = 0.0
        self._out_y = 0.0
        self._cmd   = "—"
        self._ack   = "—"
        self._running = False
        self._cmds_sent = 0

        self._draw_static()

    def _draw_static(self):
        """Draw the fixed background grid and axis labels (drawn once, kept as items)."""
        ox, oy, arm = self._ox, self._oy, self._arm

        # Grid circles
        for r in [arm//3, 2*arm//3, arm]:
            self.create_oval(ox-r, oy-r, ox+r, oy+r,
                             outline=self.GRID_COL, dash=(3,4), tags="static")

        # Axis lines
        self.create_line(ox-arm-10, oy, ox+arm+10, oy,
                         fill=self.ZERO_COL, width=1, tags="static")
        self.create_line(ox, oy-arm-10, ox, oy+arm+10,
                         fill=self.ZERO_COL, width=1, tags="static")

        # Axis labels
        for txt, x, y in [("X+", ox+arm+14, oy), ("X−", ox-arm-14, oy),
                           ("Y−", ox, oy-arm-14), ("Y+", ox, oy+arm+14)]:
            self.create_text(x, y, text=txt, fill=self.DIM_COL,
                             font=("Consolas", 8), tags="static")

        # Title
        self.create_text(ox, 10, text="MOTOR COMMAND VISUALIZER",
                         fill=self.DIM_COL, font=("Consolas", 8, "bold"), tags="static")

    def update_values(self, out_x: float, out_y: float, cmd: str, ack: str,
                      running: bool, cmds_sent: int):
        self._out_x = out_x; self._out_y = out_y
        self._cmd   = cmd;   self._ack   = ack
        self._running   = running
        self._cmds_sent = cmds_sent
        self._redraw()

    def _redraw(self):
        self.delete("dynamic")
        ox, oy, arm = self._ox, self._oy, self._arm
        max_v = self.MAX_VAL

        # ── X-axis bar (horizontal) ──────────────────────────────────────────
        x_frac = max(-1.0, min(1.0, self._out_x / max_v))
        x_len  = int(x_frac * arm)
        bw = self._bar_w // 2
        if x_len != 0:
            x0 = ox; x1 = ox + x_len
            col = self.POS_COL if x_len > 0 else self.NEG_COL
            self.create_rectangle(min(x0,x1), oy-bw, max(x0,x1), oy+bw,
                                  fill=col, outline="", tags="dynamic")
            # Arrow tip
            tip = x1; ay = oy
            sign = 1 if x_len > 0 else -1
            self.create_polygon(tip, ay-5, tip, ay+5, tip+sign*8, ay,
                                fill=col, tags="dynamic")

        # ── Y-axis bar (vertical) ────────────────────────────────────────────
        # NOTE: positive output_y tilts platform forward → ball moves up in frame
        y_frac = max(-1.0, min(1.0, self._out_y / max_v))
        y_len  = int(y_frac * arm)
        if y_len != 0:
            y0 = oy; y1 = oy + y_len
            col = self.POS_COL if y_len > 0 else self.NEG_COL
            self.create_rectangle(ox-bw, min(y0,y1), ox+bw, max(y0,y1),
                                  fill=col, outline="", tags="dynamic")
            tip = y1; sign = 1 if y_len > 0 else -1
            self.create_polygon(ox-5, tip, ox+5, tip, ox, tip+sign*8,
                                fill=col, tags="dynamic")

        # ── Origin dot ───────────────────────────────────────────────────────
        r = 5
        self.create_oval(ox-r, oy-r, ox+r, oy+r,
                         fill=self.BALL_COL, outline="", tags="dynamic")

        # ── Resultant direction arrow ─────────────────────────────────────────
        if abs(self._out_x) > 1 or abs(self._out_y) > 1:
            ex = ox + int(x_frac * arm * 0.85)
            ey = oy + int(y_frac * arm * 0.85)
            self.create_line(ox, oy, ex, ey, fill="#ffffff",
                             width=2, arrow=tk.LAST, tags="dynamic")

        # ── Bottom info bar ──────────────────────────────────────────────────
        info_y = self.H - 36

        # Command string
        cmd_txt = self._cmd if self._cmd != "—" else "no command"
        self.create_text(ox, info_y, text=f"CMD: {cmd_txt}",
                         fill=self.TEXT_COL, font=("Consolas", 9, "bold"),
                         tags="dynamic")

        # ACK / status
        ack_col = "#22c55e" if self._ack in ("OK","SENT","TEST DONE") else \
                  "#ef4444" if self._ack in ("DISCONNECTED","ERROR") else "#f59e0b"
        self.create_text(ox, info_y + 14, text=f"ACK: {self._ack}",
                         fill=ack_col, font=("Consolas", 9), tags="dynamic")

        # Running state + command count
        state_txt = f"{'▶ RUNNING' if self._running else '■ STOPPED'}   sent: {self._cmds_sent}"
        state_col = "#22c55e" if self._running else "#64748b"
        self.create_text(ox, self.H - 8, text=state_txt,
                         fill=state_col, font=("Consolas", 8), tags="dynamic")

        # X / Y numeric labels on axes
        if abs(self._out_x) > 0.5:
            lx = ox + x_len // 2
            self.create_text(lx, oy - bw - 8,
                             text=f"{self._out_x:+.1f}",
                             fill=self.TEXT_COL, font=("Consolas", 8), tags="dynamic")
        if abs(self._out_y) > 0.5:
            ly = oy + y_len // 2
            self.create_text(ox + bw + 18, ly,
                             text=f"{self._out_y:+.1f}",
                             fill=self.TEXT_COL, font=("Consolas", 8), tags="dynamic")


# =============================================================================
#  GUI APPLICATION
# =============================================================================

class GUIApp:
    BG       = "#0f1117"
    PANEL    = "#181c24"
    BORDER   = "#2a2f3d"
    ACCENT   = "#00d4ff"
    GREEN    = "#22c55e"
    RED      = "#ef4444"
    YELLOW   = "#f59e0b"
    TEAL     = "#00e5cc"
    TEXT     = "#e2e8f0"
    TEXT_DIM = "#64748b"
    BTN_BG   = "#1e2330"

    CAM_W  = 640
    CAM_H  = 480
    MASK_W = 320
    MASK_H = 120

    def __init__(self, state: AppState):
        self._state   = state
        self._running = False

        self._root = tk.Tk()
        self._root.title("Ball Balancing Platform")
        self._root.configure(bg=self.BG)
        self._root.resizable(False, False)

        self._f_title = ("Consolas", 11, "bold")
        self._f_label = ("Consolas", 10)
        self._f_value = ("Consolas", 10, "bold")
        self._f_small = ("Consolas", 9)
        self._f_btn   = ("Consolas", 10, "bold")

        self._kp_str = tk.StringVar(value=f"{CONFIG['pid_kp']:.3f}")
        self._ki_str = tk.StringVar(value=f"{CONFIG['pid_ki']:.4f}")
        self._kd_str = tk.StringVar(value=f"{CONFIG['pid_kd']:.3f}")

        self._hsv_vars = {
            "H_low":  tk.IntVar(value=CONFIG["hsv_lower"][0]),
            "S_low":  tk.IntVar(value=CONFIG["hsv_lower"][1]),
            "V_low":  tk.IntVar(value=CONFIG["hsv_lower"][2]),
            "H_high": tk.IntVar(value=CONFIG["hsv_upper"][0]),
            "S_high": tk.IntVar(value=CONFIG["hsv_upper"][1]),
            "V_high": tk.IntVar(value=CONFIG["hsv_upper"][2]),
        }
        for v in self._hsv_vars.values():
            v.trace_add("write", self._on_hsv_change)

        self._cam_photo  = None
        self._mask_photo = None

        self._build_ui()

        self._root.bind("<Escape>", lambda e: self._on_quit())
        self._root.bind("<space>",  lambda e: self._toggle_start())
        self._root.bind("r",        lambda e: self._redetect_platform())
        self._root.bind("t",        lambda e: self._test_motors())
        self._root.protocol("WM_DELETE_WINDOW", self._on_quit)

        self._refresh()

    # =========================================================================
    #  BUILD UI
    # =========================================================================

    def _build_ui(self):
        root = self._root

        title_bar = tk.Frame(root, bg=self.BG)
        title_bar.pack(fill="x", padx=10, pady=(8,4))
        tk.Label(title_bar, text="BALL BALANCING PLATFORM",
                 font=self._f_title, bg=self.BG, fg=self.ACCENT).pack(side="left")
        self._status_dot = tk.Label(title_bar, text="●", font=self._f_label,
                                    bg=self.BG, fg=self.RED)
        self._status_dot.pack(side="right", padx=(0,4))
        self._status_lbl = tk.Label(title_bar, text="STOPPED",
                                    font=self._f_label, bg=self.BG, fg=self.TEXT_DIM)
        self._status_lbl.pack(side="right")
        self._fps_lbl = tk.Label(title_bar, text="0.0 FPS",
                                 font=self._f_small, bg=self.BG, fg=self.TEXT_DIM)
        self._fps_lbl.pack(side="right", padx=16)
        tk.Frame(root, bg=self.BORDER, height=1).pack(fill="x", padx=10)

        main = tk.Frame(root, bg=self.BG)
        main.pack(fill="both", padx=10, pady=8)

        left   = self._make_panel(main)
        center = tk.Frame(main, bg=self.BG)
        right  = self._make_panel(main)

        left.pack(side="left", fill="y", padx=(0,6))
        center.pack(side="left", padx=6)
        right.pack(side="left", fill="y", padx=(6,0))

        self._build_left(left)
        self._build_center(center)
        self._build_right(right)

    # ── Left panel ─────────────────────────────────────────────────────────────

    def _build_left(self, parent):
        # Wireless camera
        self._section(parent, "WIRELESS CAMERA")
        cam_area = tk.Frame(parent, bg=self.PANEL)
        cam_area.pack(fill="x", padx=8, pady=(0,8))
        tk.Label(cam_area, text="DroidCam IP:", font=self._f_small,
                 bg=self.PANEL, fg=self.TEXT_DIM).pack(anchor="w", pady=(2,1))
        ip_row = tk.Frame(cam_area, bg=self.PANEL)
        ip_row.pack(fill="x", pady=(0,4))
        self._ip_entry = tk.Entry(ip_row, font=self._f_label, bg=self.BTN_BG,
                                  fg=self.TEXT, insertbackground=self.ACCENT,
                                  relief="flat", bd=4, width=16)
        self._ip_entry.insert(0, "e.g. 192.168.1.42")
        self._ip_entry.bind("<FocusIn>",  self._ip_entry_focus_in)
        self._ip_entry.bind("<FocusOut>", self._ip_entry_focus_out)
        self._ip_entry.bind("<Return>",   lambda e: self._connect_camera())
        self._ip_entry.pack(side="left", padx=(0,4))
        tk.Button(ip_row, text="Connect", font=self._f_btn, pady=2,
                  bg=self.ACCENT, fg="#000000", relief="flat", cursor="hand2",
                  command=self._connect_camera).pack(side="left")
        self._cam_status_lbl = tk.Label(cam_area, text="● Not connected",
                                         font=self._f_small, bg=self.PANEL, fg=self.RED)
        self._cam_status_lbl.pack(anchor="w")

        # Serial port
        self._section(parent, "SERIAL PORT (ESP32)")
        ser_area = tk.Frame(parent, bg=self.PANEL)
        ser_area.pack(fill="x", padx=8, pady=(0,8))
        tk.Label(ser_area, text="Port (e.g. COM3 or /dev/ttyUSB0):",
                 font=self._f_small, bg=self.PANEL, fg=self.TEXT_DIM,
                 wraplength=220).pack(anchor="w", pady=(2,1))
        port_row = tk.Frame(ser_area, bg=self.PANEL)
        port_row.pack(fill="x", pady=(0,4))
        self._port_entry = tk.Entry(port_row, font=self._f_label, bg=self.BTN_BG,
                                    fg=self.TEXT, insertbackground=self.ACCENT,
                                    relief="flat", bd=4, width=12)
        self._port_entry.insert(0, CONFIG["serial_port"])
        self._port_entry.pack(side="left", padx=(0,4))
        tk.Button(port_row, text="Connect", font=self._f_btn, pady=2,
                  bg="#2a3a1e", fg=self.GREEN, relief="flat", cursor="hand2",
                  command=self._connect_serial).pack(side="left")

        self._serial_status_lbl = tk.Label(ser_area, text="● Disconnected",
                                            font=self._f_small, bg=self.PANEL, fg=self.RED)
        self._serial_status_lbl.pack(anchor="w")

        # Auto-detect ports button
        tk.Button(ser_area, text="⟳ List available ports",
                  font=self._f_small, bg=self.BTN_BG, fg=self.TEXT_DIM,
                  relief="flat", cursor="hand2",
                  command=self._list_ports).pack(anchor="w", pady=(2,0))
        self._ports_lbl = tk.Label(ser_area, text="", font=self._f_small,
                                    bg=self.PANEL, fg=self.TEXT_DIM,
                                    wraplength=220, justify="left")
        self._ports_lbl.pack(anchor="w")

        # Platform detection
        self._section(parent, "PLATFORM DETECTION")
        plat_area = tk.Frame(parent, bg=self.PANEL)
        plat_area.pack(fill="x", padx=8, pady=(0,8))
        self._platform_status = tk.Label(plat_area,
                                          text="● Searching...",
                                          font=self._f_label, bg=self.PANEL,
                                          fg=self.YELLOW, wraplength=220)
        self._platform_status.pack(anchor="w", pady=(4,4))
        tk.Button(plat_area, text="↺  Re-detect Platform",
                  font=self._f_btn, pady=4, bg=self.BTN_BG, fg=self.TEAL,
                  relief="flat", cursor="hand2",
                  command=self._redetect_platform).pack(fill="x", pady=(0,4))

        # PID
        self._section(parent, "PID TUNING")
        pid_area = tk.Frame(parent, bg=self.PANEL)
        pid_area.pack(fill="x", padx=8, pady=(0,8))
        for name, strvar, key, step, fmt in [
            ("Kp", self._kp_str, "kp", CONFIG["pid_kp_step"], "%.3f"),
            ("Ki", self._ki_str, "ki", CONFIG["pid_ki_step"], "%.4f"),
            ("Kd", self._kd_str, "kd", CONFIG["pid_kd_step"], "%.3f"),
        ]:
            row = tk.Frame(pid_area, bg=self.PANEL); row.pack(fill="x", pady=4)
            tk.Label(row, text=name, font=self._f_value, bg=self.PANEL,
                     fg=self.ACCENT, width=3).pack(side="left")
            tk.Button(row, text="−", font=self._f_btn, width=2, pady=2,
                      bg=self.BTN_BG, fg=self.TEXT, relief="flat", cursor="hand2",
                      command=lambda k=key,s=step,f=fmt,v=strvar:
                          self._pid_adjust(k,-s,f,v)).pack(side="left", padx=(6,2))
            tk.Label(row, textvariable=strvar, font=self._f_value, bg=self.PANEL,
                     fg=self.TEXT, width=8, relief="sunken", bd=1).pack(side="left", padx=2)
            tk.Button(row, text="+", font=self._f_btn, width=2, pady=2,
                      bg=self.BTN_BG, fg=self.TEXT, relief="flat", cursor="hand2",
                      command=lambda k=key,s=step,f=fmt,v=strvar:
                          self._pid_adjust(k,+s,f,v)).pack(side="left", padx=(2,6))

        # HSV
        self._section(parent, "BALL HSV")
        self._hsv_note = tk.Label(parent, text="Adjust before pressing START",
                                  font=self._f_small, bg=self.PANEL, fg=self.YELLOW)
        self._hsv_note.pack(padx=8, anchor="w")
        hsv_area = tk.Frame(parent, bg=self.PANEL)
        hsv_area.pack(fill="x", padx=8, pady=(2,8))
        self._hsv_sliders = {}
        for label, key, lo, hi, color in [
            ("H min","H_low",0,179,"#ff6b6b"),("H max","H_high",0,179,"#ff6b6b"),
            ("S min","S_low",0,255,"#48cae4"),("S max","S_high",0,255,"#48cae4"),
            ("V min","V_low",0,255,"#b7e4c7"),("V max","V_high",0,255,"#b7e4c7"),
        ]:
            row = tk.Frame(hsv_area, bg=self.PANEL); row.pack(fill="x", pady=1)
            tk.Label(row, text=label, font=self._f_small, bg=self.PANEL,
                     fg=color, width=6, anchor="w").pack(side="left")
            sl = tk.Scale(row, from_=lo, to=hi, orient="horizontal",
                          variable=self._hsv_vars[key], bg=self.PANEL, fg=self.TEXT,
                          troughcolor=self.BORDER, highlightthickness=0, bd=0,
                          font=self._f_small, length=140, sliderlength=12)
            sl.pack(side="left")
            self._hsv_sliders[key] = sl

        self._ball_indicator = tk.Label(parent, text="● Ball not detected",
                                        font=self._f_label, bg=self.PANEL, fg=self.RED)
        self._ball_indicator.pack(padx=8, pady=(0,6), anchor="w")
        tk.Frame(parent, bg=self.BORDER, height=1).pack(fill="x", padx=8, pady=6)
        self._btn_start = tk.Button(parent, text="▶  START",
                                    font=self._f_btn, bg=self.GREEN, fg="#000000",
                                    activebackground="#16a34a", relief="flat",
                                    cursor="hand2", pady=8, command=self._toggle_start)
        self._btn_start.pack(fill="x", padx=8, pady=(0,8))

    # ── Center panel ───────────────────────────────────────────────────────────

    def _build_center(self, parent):
        cam_frame = self._make_panel(parent)
        cam_frame.pack(pady=(0,6))
        tk.Label(cam_frame, text="CAMERA VIEW", font=self._f_title,
                 bg=self.PANEL, fg=self.ACCENT).pack(anchor="w", padx=8, pady=4)
        self._cam_label = tk.Label(cam_frame, bg="#000000",
                                   width=self.CAM_W, height=self.CAM_H)
        self._cam_label.pack(padx=8, pady=(0,8))

        bottom_row = tk.Frame(parent, bg=self.BG)
        bottom_row.pack(fill="x")

        # Mask
        mask_frame = self._make_panel(bottom_row)
        mask_frame.pack(side="left", padx=(0,6))
        tk.Label(mask_frame, text="DETECTION MASK", font=self._f_title,
                 bg=self.PANEL, fg=self.ACCENT).pack(anchor="w", padx=8, pady=4)
        self._mask_label = tk.Label(mask_frame, bg="#000000",
                                    width=self.MASK_W, height=self.MASK_H)
        self._mask_label.pack(padx=8, pady=(0,8))

        # Motor visualizer
        viz_frame = self._make_panel(bottom_row)
        viz_frame.pack(side="left")
        tk.Label(viz_frame, text="MOTOR COMMANDS", font=self._f_title,
                 bg=self.PANEL, fg=self.ACCENT).pack(anchor="w", padx=8, pady=4)
        self._motor_viz = MotorVisualizer(viz_frame, bg=self.PANEL)
        self._motor_viz.pack(padx=8, pady=(0,4))
        # Test button under visualizer
        tk.Button(viz_frame, text="▷  Test Motors  (T key)",
                  font=self._f_small, pady=3, bg="#1a2a1a", fg=self.GREEN,
                  activebackground="#0f1f0f", relief="flat", cursor="hand2",
                  command=self._test_motors).pack(fill="x", padx=8, pady=(0,8))

    # ── Right panel ─────────────────────────────────────────────────────────────

    def _build_right(self, parent):
        self._section(parent, "BALL")
        self._t_ball_x   = self._telem_row(parent, "Position X")
        self._t_ball_y   = self._telem_row(parent, "Position Y")
        self._t_distance = self._telem_row(parent, "Distance")
        self._t_ball_st  = self._telem_row(parent, "Status")

        self._section(parent, "PID OUTPUT")
        self._t_err_x  = self._telem_row(parent, "Error X")
        self._t_err_y  = self._telem_row(parent, "Error Y")
        self._t_out_x  = self._telem_row(parent, "Output X")
        self._t_out_y  = self._telem_row(parent, "Output Y")

        self._section(parent, "MOTOR / ESP32")
        self._t_serial   = self._telem_row(parent, "Serial")
        self._t_cmd      = self._telem_row(parent, "Last Cmd")
        self._t_ack      = self._telem_row(parent, "ACK")
        self._t_sent     = self._telem_row(parent, "Total sent")

        self._section(parent, "SYSTEM")
        self._t_fps      = self._telem_row(parent, "FPS")
        self._t_loop     = self._telem_row(parent, "Loop ms")
        self._t_centre   = self._telem_row(parent, "Centre")
        self._t_plat_r   = self._telem_row(parent, "Plat radius")

    # =========================================================================
    #  HELPERS
    # =========================================================================

    def _make_panel(self, parent):
        return tk.Frame(parent, bg=self.PANEL,
                        highlightbackground=self.BORDER, highlightthickness=1)

    def _section(self, parent, title):
        tk.Label(parent, text=title, font=self._f_title,
                 bg=self.PANEL, fg=self.ACCENT).pack(anchor="w", padx=8, pady=(8,2))
        tk.Frame(parent, bg=self.BORDER, height=1).pack(fill="x", padx=8, pady=(0,4))

    def _telem_row(self, parent, label):
        row = tk.Frame(parent, bg=self.PANEL); row.pack(fill="x", padx=8, pady=1)
        tk.Label(row, text=label+":", font=self._f_small, bg=self.PANEL,
                 fg=self.TEXT_DIM, width=12, anchor="w").pack(side="left")
        val = tk.Label(row, text="—", font=self._f_value, bg=self.PANEL,
                       fg=self.TEXT, anchor="w")
        val.pack(side="left"); return val

    _PLACEHOLDER = "e.g. 192.168.1.42"

    def _ip_entry_focus_in(self, e):
        if self._ip_entry.get() == self._PLACEHOLDER:
            self._ip_entry.delete(0, "end"); self._ip_entry.config(fg=self.TEXT)

    def _ip_entry_focus_out(self, e):
        if not self._ip_entry.get().strip():
            self._ip_entry.insert(0, self._PLACEHOLDER)
            self._ip_entry.config(fg=self.TEXT_DIM)

    # =========================================================================
    #  EVENT HANDLERS
    # =========================================================================

    def _connect_camera(self):
        raw = self._ip_entry.get().strip()
        if raw == self._PLACEHOLDER or not raw: raw = ""
        with self._state.lock:
            self._state.camera_url       = raw
            self._state.reconnect_camera = True
        self._cam_status_lbl.config(text="● Connecting...", fg=self.YELLOW)

    def _connect_serial(self):
        port = self._port_entry.get().strip()
        if not port: return
        CONFIG["serial_port"] = port
        with self._state.lock:
            self._state.serial_port_used = port
            self._state.reconnect_camera = True   # triggers full reconnect incl. serial
        self._serial_status_lbl.config(text="● Connecting...", fg=self.YELLOW)
        logging.info("GUI: serial port set to %s", port)

    def _list_ports(self):
        if not SERIAL_OK:
            self._ports_lbl.config(text="pyserial not installed")
            return
        ports = serial.tools.list_ports.comports()
        if ports:
            names = ", ".join(p.device for p in ports)
            self._ports_lbl.config(text=f"Found: {names}")
        else:
            self._ports_lbl.config(text="No serial ports found")

    def _redetect_platform(self):
        with self._state.lock: self._state.redetect_platform = True
        self._platform_status.config(text="● Searching...", fg=self.YELLOW)

    def _test_motors(self):
        if self._running:
            self._serial_status_lbl.config(text="● Stop first to test", fg=self.YELLOW)
            return
        with self._state.lock: self._state.test_motors_flag = True
        logging.info("GUI: motor test requested")

    def _toggle_start(self):
        self._running = not self._running
        with self._state.lock: self._state.running = self._running
        if self._running:
            self._btn_start.config(text="■  STOP", bg=self.RED,
                                   activebackground="#b91c1c")
            self._status_dot.config(fg=self.GREEN)
            self._status_lbl.config(text="RUNNING", fg=self.GREEN)
            for sl in self._hsv_sliders.values(): sl.config(state="disabled")
            self._hsv_note.config(text="Locked during run", fg=self.TEXT_DIM)
        else:
            self._btn_start.config(text="▶  START", bg=self.GREEN,
                                   activebackground="#16a34a")
            self._status_dot.config(fg=self.RED)
            self._status_lbl.config(text="STOPPED", fg=self.TEXT_DIM)
            for sl in self._hsv_sliders.values(): sl.config(state="normal")
            self._hsv_note.config(text="Adjust before pressing START", fg=self.YELLOW)

    def _pid_adjust(self, key, step, fmt, strvar):
        with self._state.lock:
            if key == "kp":
                new = max(0.01, min(1.0, round(self._state.pid_kp+step, 4)))
                self._state.pid_kp = new
            elif key == "ki":
                new = max(0.0, min(0.05, round(self._state.pid_ki+step, 5)))
                self._state.pid_ki = new
            elif key == "kd":
                new = max(0.0, min(0.5, round(self._state.pid_kd+step, 4)))
                self._state.pid_kd = new
        strvar.set(fmt % new)

    def _on_hsv_change(self, *args):
        if self._running: return
        with self._state.lock:
            self._state.hsv_lower = [self._hsv_vars["H_low"].get(),
                                     self._hsv_vars["S_low"].get(),
                                     self._hsv_vars["V_low"].get()]
            self._state.hsv_upper = [self._hsv_vars["H_high"].get(),
                                     self._hsv_vars["S_high"].get(),
                                     self._hsv_vars["V_high"].get()]

    def _on_quit(self):
        with self._state.lock: self._state.quit_flag = True
        self._root.after(600, self._root.destroy)

    # =========================================================================
    #  REFRESH LOOP
    # =========================================================================

    def _refresh(self):
        try:
            self._update_video()
            self._update_telemetry()
        except Exception as e:
            logging.warning("GUI refresh error: %s", e)
        self._root.after(33, self._refresh)

    def _update_video(self):
        with self._state.lock:
            disp = self._state.display_frame
            mask = self._state.mask_frame
        if disp is not None:
            p = self._to_photo(disp, self.CAM_W, self.CAM_H)
            if p: self._cam_photo = p; self._cam_label.config(image=p)
        if mask is not None:
            m = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            m[mask > 0] = [0, 220, 80]
            p = self._to_photo(m, self.MASK_W, self.MASK_H)
            if p: self._mask_photo = p; self._mask_label.config(image=p)

    def _to_photo(self, frame, tw, th):
        try:
            h, w = frame.shape[:2]
            s = min(tw/w, th/h)
            r = cv2.resize(frame, (max(1,int(w*s)), max(1,int(h*s))))
            return ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(r, cv2.COLOR_BGR2RGB)))
        except: return None

    def _update_telemetry(self):
        with self._state.lock:
            bx=self._state.ball_x; by=self._state.ball_y
            dist=self._state.distance; bdet=self._state.ball_detected
            ex=self._state.error_x; ey=self._state.error_y
            ox=self._state.output_x; oy=self._state.output_y
            ser=self._state.serial_connected
            cmd=self._state.last_command; ack=self._state.ack_status
            fps=self._state.fps; lms=self._state.loop_ms
            cx=self._state.centre_x; cy=self._state.centre_y
            pr=self._state.platform_r; locked=self._state.centre_locked
            kp=self._state.pid_kp; ki=self._state.pid_ki; kd=self._state.pid_kd
            cam_ok=self._state.camera_connected
            csent=self._state.commands_sent

        # Ball
        self._t_ball_x.config(text=f"{bx:.1f} px")
        self._t_ball_y.config(text=f"{by:.1f} px")
        tol = CONFIG["tolerance_px"]
        dc = self.GREEN if dist<tol else (self.YELLOW if dist<80 else self.RED)
        self._t_distance.config(text=f"{dist:.1f} px", fg=dc)
        if bdet:
            self._t_ball_st.config(text="● DETECTED", fg=self.GREEN)
            self._ball_indicator.config(text="● Ball detected", fg=self.GREEN)
        else:
            self._t_ball_st.config(text="● LOST", fg=self.RED)
            self._ball_indicator.config(text="● Ball not detected", fg=self.RED)

        # PID
        self._t_err_x.config(text=f"{ex:+.2f}", fg=self.GREEN if abs(ex)<10 else self.TEXT)
        self._t_err_y.config(text=f"{ey:+.2f}", fg=self.GREEN if abs(ey)<10 else self.TEXT)
        self._t_out_x.config(text=f"{ox:+.2f}")
        self._t_out_y.config(text=f"{oy:+.2f}")

        # Motor / serial
        self._t_serial.config(text="● CONNECTED" if ser else "● DISCONNECTED",
                              fg=self.GREEN if ser else self.RED)
        self._t_cmd.config(text=cmd)
        self._t_ack.config(text=ack,
                           fg=self.GREEN if ack in ("OK","SENT","TEST DONE") else self.RED)
        self._t_sent.config(text=str(csent),
                            fg=self.GREEN if csent > 0 else self.TEXT_DIM)

        # System
        self._t_fps.config(text=f"{fps:.1f}", fg=self.GREEN if fps>20 else self.YELLOW)
        self._t_loop.config(text=f"{lms:.1f} ms")
        self._t_centre.config(text=f"({int(cx)},{int(cy)}) {'LOCKED' if locked else '...'}",
                              fg=self.GREEN if locked else self.YELLOW)
        self._t_plat_r.config(text=f"{int(pr)} px", fg=self.GREEN if locked else self.YELLOW)

        # Platform status
        if locked:
            self._platform_status.config(
                text=f"● LOCKED  ({int(cx)},{int(cy)})  r={int(pr)}px", fg=self.TEAL)
        else:
            self._platform_status.config(text="● Searching...", fg=self.YELLOW)

        # Camera status
        self._cam_status_lbl.config(
            text="● Connected" if cam_ok else "● Not connected",
            fg=self.GREEN if cam_ok else self.RED)

        # Serial status
        self._serial_status_lbl.config(
            text=f"● Connected ({CONFIG['serial_port']})" if ser else "● Disconnected",
            fg=self.GREEN if ser else self.RED)

        # Update motor visualizer
        self._motor_viz.update_values(ox, oy, cmd, ack, self._running, csent)

        self._fps_lbl.config(text=f"{fps:.1f} FPS")
        self._kp_str.set(f"{kp:.3f}"); self._ki_str.set(f"{ki:.4f}"); self._kd_str.set(f"{kd:.3f}")

    def run(self):
        self._root.mainloop()


# =============================================================================
#  ENTRY POINT
# =============================================================================

def main():
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(message)s",
                        datefmt="%H:%M:%S")
    state = AppState(
        pid_kp=CONFIG["pid_kp"], pid_ki=CONFIG["pid_ki"], pid_kd=CONFIG["pid_kd"],
        hsv_lower=list(CONFIG["hsv_lower"]), hsv_upper=list(CONFIG["hsv_upper"]),
        camera_url=CONFIG.get("camera_url",""),
        serial_port_used=CONFIG["serial_port"],
    )
    vision = VisionThread(state)
    vision.start()
    GUIApp(state).run()
    vision.join(timeout=3.0)


if __name__ == "__main__":
    main()