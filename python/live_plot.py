import re
import numpy as np
import matplotlib.pyplot as plt
import serial
from collections import deque

# === Einstellungen ===
SERIAL_PORT = "/dev/tty.usbmodem0010506132231"  # anpassen!
#SERIAL_PORT = "/dev/tty.usbmodem0010506051241"
BAUDRATE = 115200
MAX_SAMPLES = 45
FREQ_HZ = 2.400e9
ANT_DIST = 0.05

MAX_HISTORY = 100  # Anzahl gespeicherter Winkelwerte
angle_history = deque(maxlen=MAX_HISTORY)
frame_count = 0
WINDOW_SIZE = 15  # Anzahl der letzten Winkel für Mittelwert



# === Parsing-Funktion ===
def parse_data(line):
    match = re.match(r"IQ\[(\d+)\]: (-?\d+), (-?\d+)", line)
    if match:
        i = int(match.group(2))
        q = int(match.group(3))
        return complex(i, q)
    return None

# === CFO-Schätzung ===
def estimate_cfo(iq_block):
    phase_diff = np.angle(np.mean(iq_block[1:8] * np.conj(iq_block[0:7])))
    #phase_diff = np.angle(np.mean(iq_block[9:] * np.conj(iq_block[8:-1])))
    return phase_diff

# === CFO-Korrektur ===
def apply_cfo_correction(iq_block, delta_phi):
    n = np.arange(len(iq_block))
    return iq_block * np.exp(-1j * delta_phi * n)

# === AoA-Berechnung ===
def estimate_aoa(iq_corrected, freq=FREQ_HZ, d=ANT_DIST):
    c = 299_792_458
    iq1 = iq_corrected[::2]
    iq2 = iq_corrected[1::2]
    n = min(len(iq1), len(iq2))
    delta_phases = np.angle(iq2[:n]) - np.angle(iq1[:n])
    delta_phases = np.arctan2(np.sin(delta_phases), np.cos(delta_phases))
    delta_phi = np.mean(delta_phases)
    argument = (c * delta_phi) / (2 * np.pi * freq * d)
    argument = np.clip(argument, -1.0, 1.0)
    theta_rad = np.arcsin(argument)
    return np.degrees(theta_rad)

# === Plot vorbereiten ===
plt.ion()
fig, (ax_scatter, ax_angle, ax_avg) = plt.subplots(1, 3, figsize=(16, 5))

# Scatterplot
scatter, = ax_scatter.plot([], [], 'ro')
text = ax_scatter.text(0.05, 0.95, '', transform=ax_scatter.transAxes)
ax_scatter.set_title("IQ-Scatter")
ax_scatter.set_xlabel("I")
ax_scatter.set_ylabel("Q")
ax_scatter.grid(True)
ax_scatter.axis("equal")

# Winkel-Zeitverlauf
line_angle, = ax_angle.plot([], [], 'b-', label="Winkel")
trend_line, = ax_angle.plot([], [], 'r--', label="Trend")
ax_angle.set_title("AoA Verlauf")
ax_angle.set_xlabel("Frame")
ax_angle.set_ylabel("Winkel (°)")
ax_angle.grid(True)
ax_angle.legend()

# Zeitverlauf Winkel
line_angle, = ax_angle.plot([], [], 'b-', label="Winkel (roh)")
line_avg, = ax_angle.plot([], [], 'g-', label=f"Gleitender Mittelwert ({WINDOW_SIZE})")
trend_line, = ax_angle.plot([], [], 'r--', label="Trend")
ax_angle.set_title("AoA Verlauf")
ax_angle.set_xlabel("Frame")
ax_angle.set_ylabel("Winkel (°)")
ax_angle.grid(True)
ax_angle.legend()



def update_plot(iq_data, aoa_deg):
    global frame_count

    # IQ-Scatter aktualisieren
    scatter.set_data(iq_data.real, iq_data.imag)
    maxval = np.max(np.abs(iq_data)) + 5
    ax_scatter.set_xlim(-maxval, maxval)
    ax_scatter.set_ylim(-maxval, maxval)
    text.set_text(f"AoA: {aoa_deg:.1f}°")

    # Zeitverlauf Winkel aktualisieren
    angle_history.append(aoa_deg)
    frames = np.arange(len(angle_history))
    line_angle.set_data(frames, list(angle_history))
    ax_angle.set_xlim(0, MAX_HISTORY)
    ax_angle.set_ylim(-100, 100)

    # Trendlinie berechnen und plotten
    if len(frames) > 2:
        coeffs = np.polyfit(frames, list(angle_history), deg=1)  # lineare Regression
        trend = np.polyval(coeffs, frames)
        trend_line.set_data(frames, trend)
    # Zeitverlauf: aktuelle Rohwinkel-Daten
    angle_history.append(aoa_deg)
    winkel_roh = list(angle_history)
    frames = np.arange(len(winkel_roh))
    line_angle.set_data(frames, winkel_roh)

    # Trendlinie (lineare Regression)
    if len(frames) > 2:
        coeffs = np.polyfit(frames, winkel_roh, deg=1)
        trend = np.polyval(coeffs, frames)
        trend_line.set_data(frames, trend)
    else:
        trend_line.set_data([], [])

    # Gleitender Mittelwert berechnen (Moving Average)
    if len(winkel_roh) >= WINDOW_SIZE:
        avg_values = np.convolve(winkel_roh, np.ones(WINDOW_SIZE)/WINDOW_SIZE, mode='valid')
        avg_frames = np.arange(len(avg_values)) + (WINDOW_SIZE - 1)
        line_avg.set_data(avg_frames, avg_values)
    else:
        line_avg.set_data([], [])

    # Achsenbereich
    ax_angle.set_xlim(0, MAX_HISTORY)
    ax_angle.set_ylim(-100, 100)


    # Plot aktualisieren
    fig.canvas.draw()
    fig.canvas.flush_events()
    frame_count += 1

    



# === Serial Listener ===
def run_serial_aoa():
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.5)
    iq_buffer = []
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if line.startswith("CTE["):
            iq_buffer = []
        elif line.startswith("IQ["):
            sample = parse_data(line)
            if sample is not None:
                iq_buffer.append(sample)

        if len(iq_buffer) >= MAX_SAMPLES:
            iq = np.array(iq_buffer[:MAX_SAMPLES], dtype=np.complex64)

            # CFO berechnen aus Referenzsamples (1–8)
            delta_phi = estimate_cfo(iq)

            # Korrektur anwenden
            iq_corr = apply_cfo_correction(iq, delta_phi)

            # Amplitude entfernen
            iq_norm = iq_corr / np.abs(iq_corr)

            # AoA berechnen
            aoa_deg = estimate_aoa(iq_norm)

            # Plot aktualisieren
            update_plot(iq_corr, aoa_deg)

            # warte auf nächsten Block
            iq_buffer = []

# === Start ===
if __name__ == "__main__":
    print("Starte Live-AoA...")
    run_serial_aoa()
