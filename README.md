# 🤖 Self-Balancing Robot - Detailed Code Analysis

## 🧠 Overview

This code implements a self-balancing robot using an MPU6050 IMU sensor and BTS7960 motor drivers. The robot maintains upright balance through sensor fusion, PID control, and adaptive control zones. In addition, it integrates a Proximal Policy Optimization (PPO) based reinforcement learning model that enhances control under dynamic conditions.

---

## 🛠️ Hardware Components

### 1. MPU6050 IMU Sensor

* **Address**: `0x68` (I2C)
* **Accelerometer**: ±2g range (16384 LSB/g)
* **Gyroscope**: ±250°/sec range (131 LSB/°/sec)
* **Registers Used**:

  * `0x6B`: Power management
  * `0x1C`: Accelerometer configuration
  * `0x1B`: Gyroscope configuration
  * `0x3B`: Data start register

### 2. BTS7960 Motor Drivers

**Motor A (Left):**

* RPWM\_A (Pin 3): Right PWM for forward
* LPWM\_A (Pin 5): Left PWM for backward
* REN\_A (Pin 7): Right enable
* LEN\_A (Pin 8): Left enable

**Motor B (Right):**

* RPWM\_B (Pin 6): Right PWM for forward
* LPWM\_B (Pin 9): Left PWM for backward
* REN\_B (Pin 10): Right enable
* LEN\_B (Pin 11): Left enable

---

## 🧮 Mathematical Analysis

### 1. Sensor Calibration

```cpp
AccXoffset = Σ(AccX_raw) / 2000_samples
GyroYoffset = Σ(GyroY_raw) / 2000_samples
AccZoffset = Σ(AccZ_raw) / 2000_samples - 16384
```

* Removes sensor bias and drift
* `16384` subtraction: Removes 1g gravity component from Z-axis
* Target angle: `targetAngle = atan2(AccX_cal, AccZ_cal) × 180/π`

### 2. Angle Calculation (Sensor Fusion)

```cpp
AccX_cal = (AccX_raw - AccXoffset) / 16384.0  // Convert to g
AccZ_cal = (AccZ_raw - AccZoffset) / 16384.0  
GyroY_cal = (GyroY_raw - GyroYoffset) / 131.0 // °/sec
```

#### Complementary Filter:

```cpp
accAngle = atan2(AccX_cal, AccZ_cal) × 180/π
gyroAngle = currentAngle + (GyroY_cal × deltaTime)
currentAngle = 0.97 × gyroAngle + 0.03 × accAngle
```

#### Additional Smoothing:

```cpp
smoothedAngle = 0.75 × prev_smoothedAngle + 0.25 × currentAngle
```

### 3. PID Control Mathematics

```cpp
error = currentAngle - targetAngle
PID_output = Kp×error + Ki×∫error×dt + Kd×(derror/dt)
```

#### Control Zones:

* **Dead Zone (±0.09°)**: `error = 0`
* **Center Zone (±0.8°)**: Boosted gains
* **Moderate (0.8°–5°)**: Base gains
* **Large (5°–15°)**: Reduced gains
* **Extreme (>15°)**: Conservative gains

#### Integral Management:

```cpp
integral += error × deltaTime
integral = constrain(integral, ±integralLimit)
```

#### Derivative Filtering:

```cpp
derivative = (error - prevError) / deltaTime
filteredDerivative = 0.7×prev + 0.3×current
```

#### Velocity Damping:

```cpp
angularVelocity = (currentAngle - prevAngle) / deltaTime
velocityDampingTerm = -dampingFactor × Kp × angularVelocity
```

### 4. Motor Control Logic

```cpp
motorSpeed = (int)PID_output
```

* Forward: `analogWrite(RPWM, speed); analogWrite(LPWM, 0);`
* Backward: `analogWrite(RPWM, 0); analogWrite(LPWM, speed);`
* Stop if |angle| > 25°

---

## 🔁 Control Flow Analysis

### Setup Phase

1. I2C and motor setup
2. Calibration (2000 samples)
3. Target angle computed

### Loop Phase (every \~8ms)

1. Read serial PPO input from Pi (if available)
2. Compute deltaTime
3. Read IMU
4. Calculate angle
5. Compute PID
6. Control motors
7. Debug print every 100ms

---

## 🧠 PPO Integration (Reinforcement Learning)

### How PPO Works in This Project

* PPO is a reinforcement learning algorithm that learns optimal actions from environment feedback.
* We use real-world data (angle, error, etc.) logged during balancing trials.

### 1. Data Logging (from Pi)

* A script `ppo_live_control.py` runs on the Pi.
* It reads real-time `angle` and `error` from Arduino via Serial.
* Observations are sent to a trained PPO model:

```python
obs = np.array([error], dtype=np.float32).reshape(1, -1)
action, _ = model.predict(obs, deterministic=True)
```

* Action (float between `-0.5` and `0.5`) is sent back to Arduino via Serial.

### 2. PPO Action Handling (Arduino Side)

```cpp
if (Serial.available()) {
  String input = Serial.readStringUntil('\n');
  float ppo_adjust = input.toFloat();
  targetAngle += ppo_adjust * 2.0; // adjust angle
}
```

### 3. Training the PPO Model

* After logging, run `extract_training_data.py` to prepare:

```python
obs = df['error'].values.reshape(-1, 1)
actions = df['ppo_avg'].values.reshape(-1, 1)
```

* Train with `train_real_ppo.py` using SB3 PPO:

```python
model = PPO("MlpPolicy", env, ...)
model.learn(total_timesteps=20000)
```

* Save as `ppo_realbot_trained.zip`

### PPO Action Output

* **Range**: Typically between `-0.5` and `0.5`
* **Meaning**:

  * Negative: lean more backward
  * Positive: lean more forward
* **Effect**: Adjusts the `targetAngle` in real-time to pre-emptively counteract tilt

---

## 📟 Outputs and Diagnostics

### Serial Output (Every 100ms)

```
Angle: XX.XX | Error: XX.XX | PID: XXX.XX | Speed: XXX | Zone: XXXX | Vel: XXX.X
```

**Zones:**

* DEAD: ±0.09°
* CENTER: ±0.8°
* Normal: 0.8°–5°
* Large: 5°–15°
* EXTREME: >15°

---

## 🚀 Performance Highlights

* **Loop Frequency**: \~125Hz
* **Debug Rate**: 10Hz
* **Angle Range**: ±30°
* **Speed Range**: 0–255 PWM
* **Safety**: Cut-off if tilt > 25°
* **Reinforcement**: PPO correction layered over PID

This hybrid PID + PPO control strategy combines deterministic feedback with adaptive machine learning, creating a robust and intelligent self-balancing robot 🤖✨
