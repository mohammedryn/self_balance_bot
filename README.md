# 🤖 Assistive Walker

A real-world self-balancing robot using PID control on Arduino and PPO-based reinforcement learning on Raspberry Pi. Built for learning and experimentation, with real-time feedback, training, and logging.

---

## 📁 Project Structure and File Guide

### 🔧 Arduino Firmware (Robot Control)

| File/Function | Description |
|---------------|-------------|
| `main.ino` | Main Arduino sketch for PID balance, sensor reading, and motor driving. |
| `calibrateMPU6050()` | Calibrates MPU6050 accelerometer/gyro at boot. |
| `driveMotorsSmooth()` | Applies PID output to motors with smoothing and shaping. |
| `Serial.parseFloat()` | Receives PPO setpoint offset from Pi. |

---

### 🧠 Raspberry Pi PPO Interface

| File | Description |
|------|-------------|
| `ppo_live_control.py` | Reads angle from Arduino, runs PPO model, sends adjustment. Also logs data to CSV. |
| `ppo_log.csv` | Live logged data: angle, target, PPO actions, and adjustments. |
| `plot_log.py` | Generates graphs from the CSV for analysis. |

---

### 🧪 Real-World PPO Training

| File | Description |
|------|-------------|
| `extract_training_data.py` | Processes `ppo_log.csv` into `obs` and `action` training pairs. |
| `realbot_env.py` | Custom `gymnasium.Env` that replays real robot log for PPO training. |
| `train_real_ppo.py` | Trains PPO on real-world data using `Stable-Baselines3`. |
| `test_env.py` | Sanity test for the Gym environment. |

---

### 🧠 PPO Model Files

| File | Description |
|------|-------------|
| `ppo_realbot_trained.zip` | PPO model trained on real robot data. Used live. |
| `BaseConfig_PPO_AW_01.zip` | (Optional) PPO model trained in simulation (no longer used). |

---

## 🚀 How It Works

1. Arduino runs PID control using MPU6050 angle input.
2. Raspberry Pi receives angle, runs PPO inference, and sends offset to Arduino.
3. Arduino adjusts PID setpoint using PPO suggestion.
4. Live logging is used to refine and retrain better PPO models.

---

## 🛠️ Requirements

- Arduino Nano
- Raspberry Pi 3B+
- MPU6050 IMU
- BTS7960 motor drivers
- Python 3.8+
- Libraries: `stable-baselines3`, `gymnasium`, `numpy`, `pandas`, `matplotlib`

---

## 📦 Run Instructions

1. Upload Arduino firmware.
2. Connect Nano to Pi over USB.
3. Power on bot and SSH into Pi.
4. Activate virtualenv and run:
   ```bash
   source venv/bin/activate
   python ppo_live_control.py
