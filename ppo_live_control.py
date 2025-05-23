import serial
import time
import re
import csv
import numpy as np
from stable_baselines3 import PPO

model = PPO.load("ppo_realbot_trained.zip")
arduino = serial.Serial('/dev/ttyUSB0', 115200)
time.sleep(2)

log_file = open("ppo_log.csv", mode='w', newline='')
csv_writer = csv.writer(log_file)
csv_writer.writerow(["time", "angle", "target", "diff", "ppo_action", "adjustment"])

print("Logging started...")

while True:
    try:
        line = arduino.readline().decode().strip()

        match = re.search(r'Angle:\s*([-+]?[0-9]*\.?[0-9]+)\s+Target:\s*([-+]?[0-9]*\.?[0-9]+)', line)
        if match:
            angle = float(match.group(1))
            target = float(match.group(2))
            diff = angle - target
            obs = np.array([diff], dtype=np.float32).reshape(1, -1)


            action, _ = model.predict(obs, deterministic=True)
            ppo_adjust = float(action[0])  # single output now
            ppo_adjust = np.clip(ppo_adjust, -0.5, 0.5)

            timestamp = time.time()
            csv_writer.writerow([timestamp, angle, target, diff, action, ppo_adjust])
            log_file.flush()

            arduino.write(f"{ppo_adjust:.2f}\n".encode())

            print(f"[{timestamp:.1f}] Angle: {angle:.2f}, Diff: {diff:.2f}, PPO: {ppo_adjust:.2f}")

    except Exception as e:
        print("Error:", e)
        break

log_file.close()
