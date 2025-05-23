import serial
import re

arduino = serial.Serial('/dev/ttyUSB0', 115200)
print("Listening for data...")

while True:
    try:
        line = arduino.readline().decode().strip()

        # Regex to extract angle and diff
        match = re.search(r'Angle:\s*([-+]?[0-9]*\.?[0-9]+)\s+Target:\s*([-+]?[0-9]*\.?[0-9]+)', line)
        if match:
            angle = float(match.group(1))
            target = float(match.group(2))
            diff = angle - target
            print(f"[Parsed] Angle: {angle:.2f}, Target: {target:.2f}, Diff: {diff:.2f}")
        
        # Optional: parse motor debug lines
        if "Forward:" in line or "Reverse:" in line:
            print(f"[Motor] {line}")

    except Exception as e:
        print("Error:", e)
