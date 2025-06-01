#include <Wire.h>

// MPU6050 registers
const int MPU = 0x68;
int16_t AccX, AccY, AccZ, Temp, GyroX, GyroY, GyroZ;

// BTS7960 Motor Driver pins
// Motor A (Left)
#define RPWM_A 3   // Right PWM
#define LPWM_A 5   // Left PWM
#define REN_A 7    // Right Enable
#define LEN_A 8    // Left Enable

// Motor B (Right)  
#define RPWM_B 6   // Right PWM
#define LPWM_B 9   // Left PWM
#define REN_B 10   // Right Enable
#define LEN_B 11   // Left Enable

// Calibration variables
float AccXoffset = 0, AccYoffset = 0, AccZoffset = 0;
float GyroXoffset = 0, GyroYoffset = 0, GyroZoffset = 0;
float targetAngle = 0;  // Target balance angle after calibration
bool calibrated = false;

// Angle calculation variables
float accAngle = 0;
float gyroAngle = 0;
float currentAngle = 0;
float prevAngle = 0;

// PID variables - TUNED FOR STIFF BUT STABLE CENTER
float Kp = 7.3;    // Higher base gain for stiffness
float Ki = 0.15;   // Low integral to prevent windup
float Kd = 1.8;    // Higher derivative for damping

float error = 0;
float prevError = 0;
float integral = 0;
float derivative = 0;
float PIDoutput = 0;

// Timing variables
unsigned long prevTime = 0;
float deltaTime = 0;

// Motor speed variables
int motorSpeed = 0;
int maxSpeed = 255;  // Maximum PWM value (0-255)
int minSpeed = 10;   // Minimum motor speed for movement

// TUNED: Center zone parameters - stiff but stable
float centerTolerance = 0.8;      // Tighter center zone for more stiffness
float deadZone = 0.09;            // Smaller dead zone - only ignore very tiny movements
bool inCenterZone = false;
float velocityDamping = 0.20;     // Increased base damping

// Progressive control zones
float moderateAngle = 5.0;         // Moderate tilt threshold
float extremeAngle = 15.0;         // Extreme tilt threshold

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C communication
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up MPU6050
  Wire.endTransmission(true);
  
  // Configure accelerometer (+/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x00);  // +/- 2g full scale range
  Wire.endTransmission(true);
  
  // Configure gyroscope (+/- 250 degrees/sec)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x00);  // +/- 250 degrees/sec full scale range
  Wire.endTransmission(true);
  
  // Initialize motor driver pins
  pinMode(RPWM_A, OUTPUT);
  pinMode(LPWM_A, OUTPUT);
  pinMode(REN_A, OUTPUT);
  pinMode(LEN_A, OUTPUT);
  
  pinMode(RPWM_B, OUTPUT);
  pinMode(LPWM_B, OUTPUT);
  pinMode(REN_B, OUTPUT);
  pinMode(LEN_B, OUTPUT);
  
  // Enable motor drivers
  digitalWrite(REN_A, HIGH);
  digitalWrite(LEN_A, HIGH);
  digitalWrite(REN_B, HIGH);
  digitalWrite(LEN_B, HIGH);
  
  // Stop motors initially
  stopMotors();
  
  Serial.println("Self-Balancing Robot Initializing...");
  delay(2000);
  
  // Perform calibration
  calibrateSensors();
  
  Serial.println("Calibration Complete!");
  Serial.println("Robot ready for STIFF and STABLE center balance!");
  delay(1000);
  
  prevTime = millis();
}

void loop() {
  // === Read PPO Adjustment from Pi over Serial ===
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    float ppo_adjust = input.toFloat();
    targetAngle += ppo_adjust * 0.5;  // Scale PPO adjustment
  }

  // Calculate time difference with protection against zero/negative values
  unsigned long currentTime = millis();
  deltaTime = (currentTime - prevTime) / 1000.0; // Convert to seconds

  // Ensure minimum deltaTime to prevent division by zero
  if (deltaTime <= 0 || deltaTime > 0.05) {
    deltaTime = 0.005; // 5ms default - slightly slower for stability
  }

  prevTime = currentTime;

  // Read sensor data
  readMPU6050();

  // Calculate current angle
  calculateAngle();

  // Calculate PID with smooth center control
  calculatePID();

  // Apply motor control
  controlMotors();

  // Debug output every 100ms
  static unsigned long lastPrint = 0;
  if (currentTime - lastPrint > 100) {
    lastPrint = currentTime;
    Serial.print("Angle: ");
    Serial.print(currentAngle, 2);
    Serial.print(" | Error: ");
    Serial.print(error, 2);
    Serial.print(" | PID: ");
    Serial.print(PIDoutput, 2);
    Serial.print(" | Speed: ");
    Serial.print(motorSpeed);
    Serial.print(" | Zone: ");
    if (abs(error) <= deadZone) Serial.print("DEAD");
    else if (inCenterZone) Serial.print("CENTER");
    else if (abs(error) <= 5.0) Serial.print("Normal");
    else if (abs(error) <= 15.0) Serial.print("Large");
    else Serial.print("EXTREME");
    Serial.print(" | Vel: ");
    Serial.println((currentAngle - prevAngle) / deltaTime, 1);
  }

  delay(8); // Slightly slower loop for stability
}


void calibrateSensors() {
  Serial.println("Starting calibration...");
  Serial.println("Keep the robot upright and still!");
  
  const int samples = 2000;
  float AccX_sum = 0, AccY_sum = 0, AccZ_sum = 0;
  float GyroX_sum = 0, GyroY_sum = 0, GyroZ_sum = 0;
  
  for (int i = 0; i < samples; i++) {
    readMPU6050();
    
    AccX_sum += AccX;
    AccY_sum += AccY;
    AccZ_sum += AccZ;
    GyroX_sum += GyroX;
    GyroY_sum += GyroY;
    GyroZ_sum += GyroZ;
    
    if (i % 200 == 0) {
      Serial.print("Calibrating... ");
      Serial.print((i * 100) / samples);
      Serial.println("%");
    }
    
    delay(2);
  }
  
  // Calculate offsets
  AccXoffset = AccX_sum / samples;
  AccYoffset = AccY_sum / samples;
  AccZoffset = AccZ_sum / samples - 16384; // Remove 1g from Z-axis
  GyroXoffset = GyroX_sum / samples;
  GyroYoffset = GyroY_sum / samples;
  GyroZoffset = GyroZ_sum / samples;
  
  // Calculate target angle (current upright position)
  float accX_cal = (AccX - AccXoffset) / 16384.0;
  float accZ_cal = (AccZ - AccZoffset) / 16384.0;
  targetAngle = atan2(accX_cal, accZ_cal) * 180.0 / PI;
  
  calibrated = true;
  
  Serial.println("Calibration values:");
  Serial.print("Target Angle: "); Serial.println(targetAngle);
  Serial.print("AccX offset: "); Serial.println(AccXoffset);
  Serial.print("GyroY offset: "); Serial.println(GyroYoffset);
}

void readMPU6050() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Starting register for accelerometer
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);
  
  AccX = Wire.read() << 8 | Wire.read();
  AccY = Wire.read() << 8 | Wire.read();
  AccZ = Wire.read() << 8 | Wire.read();
  Temp = Wire.read() << 8 | Wire.read();
  GyroX = Wire.read() << 8 | Wire.read();
  GyroY = Wire.read() << 8 | Wire.read();
  GyroZ = Wire.read() << 8 | Wire.read();
}

void calculateAngle() {
  if (!calibrated) return;
  
  // Apply calibration offsets
  float accX_cal = (AccX - AccXoffset) / 16384.0; // Convert to g
  float accZ_cal = (AccZ - AccZoffset) / 16384.0;
  float gyroY_cal = (GyroY - GyroYoffset) / 131.0; // Convert to degrees/sec
  
  // Calculate angle from accelerometer
  accAngle = atan2(accX_cal, accZ_cal) * 180.0 / PI;
  
  // Calculate angle from gyroscope (integration)
  gyroAngle = currentAngle + (gyroY_cal * deltaTime);
  
  // Balanced complementary filter
  currentAngle = 0.97 * gyroAngle + 0.03 * accAngle;
  
  // Light smoothing to reduce noise without affecting responsiveness
  static float smoothedAngle = 0;
  smoothedAngle = 0.75 * smoothedAngle + 0.25 * currentAngle;
  currentAngle = smoothedAngle;
}

void calculatePID() {
  // Calculate error
  error = currentAngle - targetAngle;
  
  // NEW: Apply dead zone to prevent micro-oscillations
  if (abs(error) <= deadZone) {
    error = 0; // Ignore tiny movements
  }
  
  // Check if we're in the center zone
  inCenterZone = (abs(error) <= centerTolerance);
  
  // Calculate angular velocity for damping
  float angularVelocity = 0;
  if (deltaTime > 0.001) {
    angularVelocity = (currentAngle - prevAngle) / deltaTime;
  }
  prevAngle = currentAngle;
  
  // FIXED: Gentler progressive PID gains
  float currentKp = Kp;
  float currentKi = Ki;
  float currentKd = Kd;
  
  float absError = abs(error);
  
  if (absError <= deadZone) {
    // In dead zone - no correction needed
    currentKp = 0;
    currentKi = 0;
    currentKd = 0;
  }
  else if (inCenterZone) {
    // BALANCED: Stiff but controlled center response
    currentKp *= 2.5; // Significant stiffness increase
    currentKi *= 0.3;  // Reduce integral to prevent oscillation
    currentKd *= 2.0;  // Increase damping to control stiffness
  } 
  else if (absError <= moderateAngle) {
    // Moderate angles - normal gains
    // Keep gains as is
  }
  else if (absError <= extremeAngle) {
    // Large angles - reduce gains for stability
    currentKp *= 0.8;  // Less aggressive reduction
    currentKi *= 0.6;  // Moderate integral reduction
    currentKd *= 1.2;  // Moderate damping increase
  }
  else {
    // Extreme angles - conservative gains
    currentKp *= 0.6;  
    currentKi *= 0.3;  
    currentKd *= 1.4;  
  }
  
  // Proportional term
  float proportional = currentKp * error;
  
  // Integral term with smart limiting
  if (abs(error) > deadZone) {
    integral += error * deltaTime;
  } else {
    integral *= 0.95; // Slowly decay integral in dead zone
  }
  
  // Progressive integral limits
  float integralLimit = 30.0;
  if (absError > moderateAngle) {
    integralLimit = 15.0;
  }
  if (absError > extremeAngle) {
    integralLimit = 8.0;
  }
  
  integral = constrain(integral, -integralLimit, integralLimit);
  float integralTerm = currentKi * integral;
  
  // Reset integral for extreme angles or high velocity
  if (absError > extremeAngle || abs(angularVelocity) > 40.0) {
    integral *= 0.1;
  }
  
  // Derivative term with better filtering
  derivative = 0;
  if (deltaTime > 0.001) {
    derivative = (error - prevError) / deltaTime;
    derivative = constrain(derivative, -600, 600); // Limit derivative
  }
  
  // Smooth derivative filtering
  static float filteredDerivative = 0;
  filteredDerivative = 0.7 * filteredDerivative + 0.3 * derivative;
  float derivativeTerm = currentKd * filteredDerivative;
  
  // Velocity damping - strategic application
  float dampingFactor = velocityDamping;
  if (inCenterZone && abs(angularVelocity) > 15.0) {
    dampingFactor *= 1.0; // Extra damping only for fast center movements
  } else if (inCenterZone) {
    dampingFactor *= 0.9; // Slight increase for center stability
  } else if (absError > extremeAngle) {
    dampingFactor *= 2.2;
  } else if (absError > moderateAngle) {
    dampingFactor *= 1.5;
  }
  
  float velocityDampingTerm = -dampingFactor * currentKp * angularVelocity;
  
  // Calculate PID output
  PIDoutput = proportional + integralTerm + derivativeTerm + velocityDampingTerm;
  
  // ADD THIS NEW SECTION HERE:
  // STIFF CENTER LOCK - boost correction force when in center
  if (inCenterZone && abs(angularVelocity) < 10.0) {
      // In center and moving slowly - apply maximum stiffness
      PIDoutput *= 1.1;  // Boost the correction force
  }

  // Progressive output limiting with center focus
  if (absError > extremeAngle) {
    PIDoutput = constrain(PIDoutput, -120, 120);
  } else if (inCenterZone && abs(angularVelocity) > 25.0) {
    PIDoutput = constrain(PIDoutput, -100, 100); // Limit only fast center movements
  }
  
  // Safety checks for NaN
  if (isnan(PIDoutput) || isinf(PIDoutput)) {
    PIDoutput = 0;
    integral = 0;
    filteredDerivative = 0;
    Serial.println("PID Reset - NaN detected!");
  }
  
  // Update previous error
  prevError = error;
  
  // Convert PID output to motor speed
  motorSpeed = (int)PIDoutput;
  
  // Apply minimum speed only if not in dead zone
  if (abs(error) > deadZone) {
    if (abs(motorSpeed) < minSpeed && abs(motorSpeed) >= 5) {
      motorSpeed = (motorSpeed > 0) ? minSpeed : -minSpeed;
    } else if (abs(motorSpeed) < 5) {
      motorSpeed = 0;
    }
  } else {
    motorSpeed = 0; // Force stop in dead zone
  }
  
  // Constrain to maximum speed
  motorSpeed = constrain(motorSpeed, -maxSpeed, maxSpeed);
}



void controlMotors() {
  // Safety check - stop if too tilted
  if (abs(currentAngle - targetAngle) > 25) {
    stopMotors();
    integral = 0; // Reset integral when falling over
    Serial.println("Safety stop - too tilted!");
    return;
  }
  
  // Apply motor speed
  if (motorSpeed == 0) {
    stopMotors();
  } else if (motorSpeed > 0) {
    // Move forward
    moveForward(abs(motorSpeed));
  } else {
    // Move backward  
    moveBackward(abs(motorSpeed));
  }
}

void moveForward(int speed) {
  // Left motor forward
  analogWrite(RPWM_A, speed);
  analogWrite(LPWM_A, 0);
  
  // Right motor forward
  analogWrite(RPWM_B, speed);
  analogWrite(LPWM_B, 0);
}

void moveBackward(int speed) {
  // Left motor backward
  analogWrite(RPWM_A, 0);
  analogWrite(LPWM_A, speed);
  
  // Right motor backward
  analogWrite(RPWM_B, 0);
  analogWrite(LPWM_B, speed);
}

void stopMotors() {
  analogWrite(RPWM_A, 0);
  analogWrite(LPWM_A, 0);
  analogWrite(RPWM_B, 0);
  analogWrite(LPWM_B, 0);
}