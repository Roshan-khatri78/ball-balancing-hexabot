import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np

SERIAL_PORT = 'COM5'
BAUD_RATE = 115200
MAX_POINTS = 100

# Buffers
ball_x = deque(maxlen=MAX_POINTS)
ball_y = deque(maxlen=MAX_POINTS)
trace_x = deque(maxlen=MAX_POINTS)
trace_y = deque(maxlen=MAX_POINTS)
grey_trace_x = deque(maxlen=MAX_POINTS)  # For grey trace lines
grey_trace_y = deque(maxlen=MAX_POINTS)
err_x   = deque(maxlen=MAX_POINTS)
err_y   = deque(maxlen=MAX_POINTS)
pid_x   = deque(maxlen=MAX_POINTS)
pid_y   = deque(maxlen=MAX_POINTS)
prev_pid_x = deque(maxlen=MAX_POINTS)  # Previous PID values for grey display
prev_pid_y = deque(maxlen=MAX_POINTS)

current_shape = ""
shape_x, shape_y = [], []

# Initialize serial
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
time.sleep(2)

# Create figure and axes
fig, (ax_trace, ax_error, ax_pid) = plt.subplots(3, 1, figsize=(10, 12))

# --- Trace subplot ---
ax_trace.set_title("Ball Trace Path", fontsize=14, fontweight='bold')
ax_trace.set_xlim(-4, 4)
ax_trace.set_ylim(-4, 4)
ax_trace.grid(True, alpha=0.3)
ax_trace.set_xlabel("X Position")
ax_trace.set_ylabel("Y Position")

outline_ln, = ax_trace.plot([], [], color='navy', lw=2, alpha=0.8, label='Target Shape')
trail_ln,   = ax_trace.plot([], [], color='lime', lw=2.5, label='Ball Path')
grey_ln,    = ax_trace.plot([], [], color='grey', lw=1.5, alpha=0.6, linestyle='--', label='Movement Trace')
ball_ln,    = ax_trace.plot([], [], 'ro', markersize=8, label='Ball Position')
ax_trace.legend(loc='upper right', fontsize=9)

# --- Error subplot (colored with enhanced styling) ---
ax_error.set_title("PID Errors (Enhanced)", fontsize=14, fontweight='bold')
ax_error.set_xlim(0, MAX_POINTS)
ax_error.set_ylim(-4, 4)
ax_error.grid(True, alpha=0.3)
ax_error.set_xlabel("Time Steps")
ax_error.set_ylabel("Error Value")

ex_ln, = ax_error.plot([], [], color='crimson', lw=2.5, label='Error X', marker='o', markersize=2)
ey_ln, = ax_error.plot([], [], color='darkgreen', lw=2.5, label='Error Y', marker='s', markersize=2)
ax_error.legend(loc='upper right', fontsize=10)
ax_error.axhline(y=0, color='black', linestyle='-', alpha=0.3)

# --- PID output subplot (with previous in grey) ---
ax_pid.set_title("PID Output (Current vs Previous)", fontsize=14, fontweight='bold')
ax_pid.set_xlim(0, MAX_POINTS)
ax_pid.set_ylim(-50, 50)
ax_pid.grid(True, alpha=0.3)
ax_pid.set_xlabel("Time Steps")
ax_pid.set_ylabel("PID Output")

# Previous PID (grey)
px_prev_ln, = ax_pid.plot([], [], color='lightgrey', lw=1.5, alpha=0.5, linestyle=':', label='Prev PID X')
py_prev_ln, = ax_pid.plot([], [], color='darkgrey', lw=1.5, alpha=0.5, linestyle=':', label='Prev PID Y')

# Current PID (colored)
px_ln, = ax_pid.plot([], [], color='dodgerblue', lw=2.5, marker='o', markersize=2, label='Current PID X')
py_ln, = ax_pid.plot([], [], color='mediumorchid', lw=2.5, marker='s', markersize=2, label='Current PID Y')

ax_pid.axhline(y=0, color='black', linestyle='-', alpha=0.3)
ax_pid.legend(loc='upper right', fontsize=10)

plt.tight_layout()

# Function to generate shape outline
def get_shape(name):
    t = np.linspace(0, 2*np.pi, 200)
    if name == "Circle":
        return 2*np.cos(t), 2*np.sin(t)
    if name == "Triangle":
        # Proper equilateral triangle
        angles = np.array([0, 2*np.pi/3, 4*np.pi/3, 0]) + np.pi/2
        x = 2*np.cos(angles)
        y = 2*np.sin(angles)
        return x, y
    if name == "Square":
        # Proper square
        pts = np.array([[-2, -2], [2, -2], [2, 2], [-2, 2], [-2, -2]])
        return pts[:,0], pts[:,1]
    if name == "Infinity":
        return 1.5*np.sin(t), 0.75*np.sin(2*t)
    if name == "Hexagon":
        angles = np.linspace(0, 2*np.pi, 7)
        return 2*np.cos(angles), 2*np.sin(angles)
    if name == "Ellipse":
        return 3*np.cos(t), 1.5*np.sin(t)
    return [], []

# Initialize animation lines
def init():
    for ln in (outline_ln, trail_ln, grey_ln, ball_ln, ex_ln, ey_ln, px_ln, py_ln, px_prev_ln, py_prev_ln):
        ln.set_data([], [])
    return outline_ln, trail_ln, grey_ln, ball_ln, ex_ln, ey_ln, px_ln, py_ln, px_prev_ln, py_prev_ln

# Animation update function
def update(frame):
    global current_shape, shape_x, shape_y

    # Read serial input
    while ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()
        parts = line.split(',')

        if parts[0] == "CLEAR" or parts[0] == "CLEAR_TRACE":
            # Clear all traces and reset to center
            for buf in (ball_x, ball_y, trace_x, trace_y, grey_trace_x, grey_trace_y, 
                       err_x, err_y, pid_x, pid_y, prev_pid_x, prev_pid_y):
                buf.clear()
            ball_x.append(0); ball_y.append(0)
            trace_x.append(0); trace_y.append(0)
            err_x.append(0); err_y.append(0)
            pid_x.append(0); pid_y.append(0)

        elif parts[0] == "SHAPE" and len(parts) == 2:
            current_shape = parts[1]
            shape_x, shape_y = get_shape(current_shape)
            if len(shape_x) > 0:
                trace_x.clear(); trace_y.clear()
                grey_trace_x.clear(); grey_trace_y.clear()
                # Start from first point of shape
                trace_x.append(shape_x[0]); trace_y.append(shape_y[0])
                ball_x.append(shape_x[0]); ball_y.append(shape_y[0])

        elif parts[0] == "GREY_TRACE" and len(parts) == 5:
            # Grey trace from start position to target
            start_x, start_y = float(parts[1]), float(parts[2])
            end_x, end_y = float(parts[3]), float(parts[4])
            # Create line from start to end
            steps = 10
            for i in range(steps + 1):
                t = i / steps
                x = start_x + t * (end_x - start_x)
                y = start_y + t * (end_y - start_y)
                grey_trace_x.append(x)
                grey_trace_y.append(y)

        elif parts[0] in ("STATIC", "DYNAMIC") and len(parts) >= 5:
            x, y = float(parts[1]), float(parts[2])
            
            # Store previous PID values before updating
            if len(pid_x) > 0:
                prev_pid_x.append(pid_x[-1])
                prev_pid_y.append(pid_y[-1])
            
            ball_x.append(x); ball_y.append(y)
            trace_x.append(x); trace_y.append(y)
            
            # Calculate errors (target - current)
            ex, ey = -x, -y  # Simplified error calculation
            err_x.append(ex); err_y.append(ey)
            
            # PID values from servo positions
            px = float(parts[3]) - 90
            py = float(parts[4]) - 90
            pid_x.append(px); pid_y.append(py)

    # Update outline
    outline_ln.set_data(shape_x, shape_y)

    # Update trail and ball
    trail_ln.set_data(list(trace_x), list(trace_y))
    grey_ln.set_data(list(grey_trace_x), list(grey_trace_y))
    if len(ball_x) > 0:
        ball_ln.set_data([ball_x[-1]], [ball_y[-1]])

    # Update errors with enhanced colors
    xs = list(range(len(err_x)))
    ex_ln.set_data(xs, list(err_x))
    ey_ln.set_data(xs, list(err_y))
    if len(xs) > 0:
        ax_error.set_xlim(max(0, len(xs) - MAX_POINTS), max(len(xs), 1))

    # Update PID with previous values in grey
    xs_pid = list(range(len(pid_x)))
    
    # Previous PID (shifted by one step)
    if len(prev_pid_x) > 0:
        xs_prev = list(range(len(prev_pid_x)))
        px_prev_ln.set_data(xs_prev, list(prev_pid_x))
        py_prev_ln.set_data(xs_prev, list(prev_pid_y))
    
    # Current PID
    px_ln.set_data(xs_pid, list(pid_x))
    py_ln.set_data(xs_pid, list(pid_y))
    
    if len(xs_pid) > 0:
        ax_pid.set_xlim(max(0, len(xs_pid) - MAX_POINTS), max(len(xs_pid), 1))

    return outline_ln, trail_ln, grey_ln, ball_ln, ex_ln, ey_ln, px_ln, py_ln, px_prev_ln, py_prev_ln

# Start animation
ani = animation.FuncAnimation(
    fig, update, init_func=init,
    interval=50, blit=True, cache_frame_data=False
)

plt.show()