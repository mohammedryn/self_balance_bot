import serial
import time
import re
import csv
import numpy as np
from stable_baselines3 import PPO

# Load PPO model
model = PPO.load("ppo_realbot_trained.zip")

# Connect to Arduino
arduino = serial.Serial('/dev/ttyUSB0', 115200)
time.sleep(2)

# Open CSV log
log_file = open("ppo_log.csv", mode='w', newline='')
csv_writer = csv.writer(log_file)
csv_writer.writerow(["time", "angle", "error", "ppo_action", "adjustment"])

print("Logging started...")

while True:
    try:
        line = arduino.readline().decode(errors='ignore').strip()

        # Match format like: Angle: 2.56 | Error: 1.23 | PID: 38.00 ...
        match = re.search(r'Angle:\s*(-?\d+\.\d+)\s+\|\s+Error:\s*(-?\d+\.\d+)', line)
        if match:
            angle = float(match.group(1))
            error = float(match.group(2))
            obs = np.array([error], dtype=np.float32).reshape(1, -1)

            action, _ = model.predict(obs, deterministic=True)
            ppo_adjust = float(action[0])
            ppo_adjust = np.clip(ppo_adjust, -0.5, 0.5)

            timestamp = time.time()
            csv_writer.writerow([timestamp, angle, error, action, ppo_adjust])
            log_file.flush()

            arduino.write(f"{ppo_adjust:.2f}\n".encode())

            print(f"[{timestamp:.1f}] Angle: {angle:.2f}, Error: {error:.2f}, PPO: {ppo_adjust:.2f}")

    except Exception as e:
        print("Error:", e)
        break

log_file.close()
