#!/usr/bin/env python3
import serial
import sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re
import numpy as np
from aoa import estimate_aoa_with_reference

def parse_data(line):
    # IQ[21]: -128, 62
    match = re.match(r"IQ\[(\d+)\]: (-?\d+), (-?\d+)", line)
    if match:
        index = int(match.group(1))
        i = int(match.group(2))
        q = int(match.group(3))
        return index, i, q
    return None

MAX_POINTS = 45

# Make the radial limit fixed (so the point always sits on the circle)
# --- Set up figure & fixed axes ---
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim(-1.1, 1.1)
ax.set_ylim(-1.1, 1.1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.grid(True)

# draw a unit circle for reference
circle = plt.Circle((0,0), 1, color='gray', fill=False, linestyle='--')
ax.add_artist(circle)

# initial quiver (arrow) from origin pointing along +X
Q = ax.quiver(
    [0], [0],    # tail at origin
    [1], [0],    # U=1, V=0 (points right)
    angles='xy', scale_units='xy', scale=1, 
    width=0.02
)

fc  = 2.402e9   # BLE ch37
dt  = 2e-6      # 2 µs sw delay
d   = 0.05      # 5 cm ant spacing
current_aoa = 0

def serial_handler():
    global current_aoa
    # Configure serial port
    port = '/dev/ttyACM0'
    baudrate = 115200  # adjust if needed
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to {port} at {baudrate} baud.")
        
        iq = []
        # print data to console
        print("Reading data from serial port. Press Ctrl+C to stop.")
        while True:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue
            print(line)
            data = parse_data(line)
            if not data:
                continue
            index, i, q = data
            iq.append((i, q))
            if index == 44:
                # Process the complete set of IQ samples
                aoas, mean_aoa, cfo = estimate_aoa_with_reference(iq, fc, dt, d)
                print("Per‐pair AoAs (°):", aoas)
                print("Mean AoA      (°):", mean_aoa)
                print("Estimated CFO (Hz):", cfo)
                current_aoa = mean_aoa

    except serial.SerialException as e:
        print(f"Unable to open port {port}: {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        ser.close()

def update(frame):
    global current_aoa
    # 400 grads → 2π rad  ⇒ θ = grads * π/200
    theta = current_aoa * (np.pi / 200)
    u = np.cos(theta)
    v = np.sin(theta)
    Q.set_UVC([u], [v])   # update vector components
    return Q,

# ani = FuncAnimation(
#     fig, update, blit=True, interval=100  # update every 100 ms
# )

# # Start the serial handler in a separate thread
# import threading
# serial_thread = threading.Thread(target=serial_handler)
# # kill thread on exit
# serial_thread.daemon = True
# serial_thread.start()

serial_handler()  # start reading serial data

# plt.show()
