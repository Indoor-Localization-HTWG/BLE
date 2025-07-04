#!/usr/bin/env python3
import serial
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
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

fig, ax = plt.subplots()
green_scatter = ax.scatter([], [], color='green')
red_scatter = ax.scatter([], [], color='red')
green_scatter.set_offsets(np.empty((0, 2)))
red_scatter.set_offsets(np.empty((0, 2)))

MAX_POINTS = 45

def init():
    ax.set_xlim(-160, 160)
    ax.set_ylim(-160, 160)
    ax.set_xlabel('I')
    ax.set_ylabel('Q')
    ax.set_title('Live IQ Data Plotter')
    return green_scatter, red_scatter

red_data = [[], []]  # Initialize with empty lists
green_data = [[], []]  # Initialize with empty lists

fc  = 2.402e9   # BLE ch37
dt  = 2e-6      # 2 µs sw delay
d   = 0.05      # 5 cm ant spacing
dir = 0

def serial_handler():
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
                dir = mean_aoa

            if index % 2 == 0:
                # Even index: red data
                red_data[0].append(i)
                red_data[1].append(q)
            else:
                # Odd index: green data
                green_data[0].append(i)
                green_data[1].append(q)

            if len(red_data[0]) > MAX_POINTS:
                red_data[0].pop(0)
                red_data[1].pop(0)
            if len(green_data[0]) > MAX_POINTS:
                green_data[0].pop(0)
                green_data[1].pop(0)
    except serial.SerialException as e:
        print(f"Unable to open port {port}: {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        ser.close()

# start the serial handler in a separate thread
import threading
serial_thread = threading.Thread(target=serial_handler)
serial_thread.start()

# stop the thread when the program exits
import atexit
atexit.register(serial_thread.join)

def update(frame):
    red_data_np = np.array(red_data)
    green_data_np = np.array(green_data)

    # Update the scatter plots with new data
    red_scatter.set_offsets(np.column_stack((red_data_np[0], red_data_np[1])))
    green_scatter.set_offsets(np.column_stack((green_data_np[0], green_data_np[1])))
    return green_scatter, red_scatter


# Create the animation: call `update` every 200 ms
ani = animation.FuncAnimation(
    fig,
    update,
    init_func=init,
    interval=200,
    blit=True
)
plt.show()