# Self-Balancing Robot - Simple Explanation

## What Does This Robot Do?

Imagine trying to balance a broomstick on your palm. You constantly move your hand to keep it upright - that's exactly what this robot does! It uses sensors to detect when it's tilting and moves its wheels to stay balanced.

## The Hardware (Robot's Body Parts)

### 1. MPU6050 Sensor (The Robot's Inner Ear)

Think of this as the robot's balance organ, like your inner ear that tells you when you're falling.

**What it measures:**

* **Accelerometer**: "Am I tilting?" (like feeling gravity pull you sideways)
* **Gyroscope**: "How fast am I rotating?" (like sensing when you spin in a chair)

**Real-world example**: When you close your eyes and tilt your head, you still know you're tilting - that's your accelerometer. When someone spins your chair, you feel the rotation - that's your gyroscope.

### 2. Motors (The Robot's Legs)

Two motors with wheels that can spin forward or backward to "chase" the fall direction.

**Example**: If you're falling backward while balancing a stick, you step backward to catch yourself. The robot does the same by moving its wheels backward.

## The Math Made Simple

### Step 1: Reading the Sensors (Getting Information)

**Calibration (Teaching the Robot What "Straight" Means):**

```
Take 2000 readings while robot is perfectly upright
Average them out = "This is my zero position"
```

**Real-world example**: Like calibrating a scale - you put nothing on it and press "zero" so it knows what empty weighs.

**Converting Raw Numbers to Useful Information:**

```
Raw sensor value ÷ 16384 = Actual tilt in "g-forces"
Raw gyro value ÷ 131 = Actual rotation speed in degrees per second
```

### Step 2: Figuring Out the Current Angle (Sensor Fusion)

**The Problem**:

* Accelerometer: Accurate long-term, but shaky (like a nervous person pointing)
* Gyroscope: Smooth short-term, but drifts over time (like a compass that slowly goes wrong)

**The Solution - Complementary Filter:**

```
Final Angle = (97% × Gyro Angle) + (3% × Accelerometer Angle)
```

**Real-world example**: Imagine asking directions from two people:

* Person A (Gyro): Gives smooth, confident directions but gradually gets lost
* Person B (Accelerometer): Always knows the general direction but hands shake when pointing
* You mostly follow A but occasionally check with B to stay on track

### Step 3: The PID Controller (The Robot's Brain)

PID is like having three different reactions to falling:

#### P - Proportional (The Panic Response)

```
"How much am I tilting RIGHT NOW?"
If tilting 5°, push back with 5× force
If tilting 10°, push back with 10× force
```

**Example**: Like pushing harder on a heavy door - more tilt = more correction.

#### I - Integral (The Memory Response)

```
"Have I been tilting the same way for a while?"
Add up all the small tilts over time
Apply extra push to fix persistent lean
```

**Example**: If you notice you've been leaning left for 10 seconds, you consciously shift right even if you're not leaning much at that moment.

#### D - Derivative (The Prediction Response)

```
"How FAST am I starting to fall?"
If falling slowly → gentle correction
If falling fast → strong correction
```

**Example**: Like catching a glass that's sliding off a table - you react to the speed of falling, not just the position.

### Step 4: Smart Control Zones (Different Reactions for Different Situations)

The robot is like a person with different reactions based on how much they're falling:

#### Dead Zone (±0.09°) - "Ignore Tiny Wobbles"

```
If tilt < 0.09°: Do nothing
```

**Example**: You don't correct yourself for every tiny sway when standing still.

#### Center Zone (±0.8°) - "Stay Perfectly Upright!"

```
If tilt < 0.8°: STRONG correction (2.5× normal power)
```

**Example**: Like a soldier at attention - any small lean gets immediately corrected.

#### Normal Zone (0.8° to 5°) - "Standard Correction"

```
Use normal correction strength
```

**Example**: Normal walking - you correct naturally without thinking.

#### Large Tilt Zone (5° to 15°) - "Careful Recovery"

```
Use gentler correction (80% normal power)
```

**Example**: When you slip on ice, you recover slowly and carefully to avoid overcorrecting.

#### Extreme Zone (>15°) - "Emergency Mode"

```
Use very gentle correction (60% normal power)
```

**Example**: Like barely catching yourself from a bad fall - gentle movements to avoid making it worse.

## PPO AI Integration 🤖 (Robot Learns from Itself!)

We added reinforcement learning (RL) to help the robot **learn from its own balancing behavior**.

### What is PPO?

**Proximal Policy Optimization (PPO)** is an algorithm that teaches the robot how to balance by observing sensor inputs and experimenting with actions. It's like giving the robot a brain that improves over time.

### How We Trained the AI

1. 📊 **Collect real balancing data** from the running robot (angle, error, PID response)
2. 🔢 **Save observations and actions** to a CSV file
3. 🎓 **Train a PPO model** using this data in Python (with Stable Baselines3)

   * The model learns: "For this angle and error, what small correction helps?"
4. 🔖 **Export the trained model** to a file: `ppo_realbot_trained.zip`

### What the AI Outputs

* The PPO model outputs a **small float between -0.5 and +0.5**
* This is called the `ppo_adjust` value
* We scale it on Arduino by `ppo_adjust × 2.0` and add to the **target angle**

### How Pi Sends Data to Arduino

On Raspberry Pi:

```python
arduino.write(f"{ppo_adjust:.2f}\n".encode())
```

Arduino reads this via:

```cpp
if (Serial.available()) {
  String input = Serial.readStringUntil('\n');
  float ppo_adjust = input.toFloat();
  targetAngle += ppo_adjust * 2.0;
}
```

### What Does This Do?

The robot now adjusts its balance point slightly forward or backward depending on what the AI suggests.

**Result:**

* The AI may predict a better correction direction
* It improves over PID-only performance
* Makes balancing smoother in tricky situations like slopes, inertia changes, etc.

## What Actually Happens (The Output)

### The Numbers You'll See:

**Current Angle**:

* Normal operation: ±2° (tiny wobbles)
* Active balancing: ±5° (visible swaying)
* Emergency: ±25° (robot shuts off beyond this)

**Motor Speed**:

* Range: 0 to 255 (like a volume knob from 0 to max)
* Minimum: 10 (below this, motors don't have enough power to move)
* Typical: 20-100 for normal balancing

**PPO Adjust**:

* Range: \~±0.5 (from model)
* Scaled on Arduino: \~±1.0 (modifies target angle)

**Example Output on Screen**:

```
Angle: 2.34 | Error: 2.34 | PID: 45.67 | Speed: 46 | Zone: CENTER | Vel: -12.3
```

**Translation**:

* "I'm tilting 2.34° forward"
* "That's 2.34° away from perfect"
* "Brain says apply 45.67 units of correction"
* "Motors will spin at speed 46 (out of 255)"
* "I'm in the CENTER zone (strong correction)"
* "I'm tilting at -12.3°/second (falling backward)"

### Real-World Behavior:

**When Robot is Balanced**:

* Tiny corrections every 8 milliseconds (125 times per second!)
* Like making micro-adjustments to balance a pencil on your finger

**When Robot Starts to Fall**:

1. Sensor detects tilt in 8ms
2. Brain calculates correction in 1ms
3. PPO slightly shifts balance target
4. Motors respond immediately
5. Wheels "chase" the fall direction

**Example Scenario**:

```
Robot tilts 3° forward →
PID calculates "move forward at speed 73" →
PPO shifts targetAngle +0.5° →
Motors spin forward →
Robot body catches up to wheels →
Balance restored
```

## Simple Summary

This robot is essentially a very fast, very precise version of you balancing a broomstick. It:

1. ✨ **Senses** tilt 125 times per second (you do this maybe 10 times per second)
2. ⚖️ **Calculates** the perfect correction using math (you do this intuitively)
3. 🏍️ **Moves** its wheels to stay under its center of gravity (like you moving your hand under a balanced stick)
4. 🧠 **Adapts** its reaction based on how badly it's falling (gentle for small tilts, careful for big ones)
5. 🤖 **Learns** using PPO-based AI to improve corrections over time

The magic is in doing all this 125 times per second with mathematical precision AND a touch of learned intelligence ✨🤖
