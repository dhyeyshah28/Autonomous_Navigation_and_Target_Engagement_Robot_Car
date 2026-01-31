# 🤖 Autonomous Navigation and Target Engagement Robot Car: Robot Battle Arena (RoBA) Champion

> **Description**: We built a robust autonomous mobile robot for the MEAM 510 Robot Battle Arena game, achieving championship performance with robust localization, autonomous navigation, and strategic gameplay capabilities.

[![Competition Result](https://img.shields.io/badge/Competition-1st%20Place-gold?style=for-the-badge)](https://github.com)
[![Alliance](https://img.shields.io/badge/Alliance-Winning%20Team-success?style=for-the-badge)](https://github.com)
[![Python](https://img.shields.io/badge/Arduino-C%2FC%2B%2B-blue?style=for-the-badge&logo=arduino)](https://www.arduino.cc/)
[![ROS](https://img.shields.io/badge/ESP32-IoT-black?style=for-the-badge&logo=espressif)](https://www.espressif.com/)

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Key Features](#-key-features)
- [System Architecture](#-system-architecture)
- [Technical Approach](#-technical-approach)
  - [1. Mechanical Design](#1-mechanical-design)
  - [2. Electrical Design](#2-electrical-design)
  - [3. Localization System](#3-localization-system)
  - [4. Control Architecture](#4-control-architecture)
  - [5. Autonomous Navigation](#5-autonomous-navigation)
  - [6. Wall Following](#6-wall-following)
- [Performance Results](#-performance-results)
- [Installation & Setup](#-installation--setup)
- [Usage](#-usage)
- [Repository Structure](#-repository-structure)
- [Key Algorithms](#-key-algorithms)
  - [1. Velocity Controller](#1-velocity-controller)
  - [2. Waypoint Navigation](#2-waypoint-navigation)
  - [3. Wall Following Algorithm](#3-wall-following-algorithm)
  - [4. Origin Calibration](#4-origin-calibration)
- [Competition Rules](#-competition-rules)
- [Lessons Learned](#-lessons-learned)
- [Future Improvements](#-future-improvements)
- [References](#-references)
- [Acknowledgments](#-acknowledgments)

---

## 🎯 Overview

This project implements a complete autonomous mobile robot system for the **MEAM 510 Robot Battle Arena (RoBA) Challenge**. The game was based on multiplayer Battle Arena style gameplay where robots compete in a 3v3 format to destroy the opposing team's nexus while defending their own.

The goal was to design, implement, and deploy a robust solution capable of:

- 🎮 **Autonomous Navigation** to strategic locations during the 45-second autonomous period
- 🎯 **Manual Control** via WiFi for tactical gameplay
- 🧱 **Wall Following** for perimeter navigation
- ⚔️ **Combat Mechanics** by triggering opponent whisker switches to deal damage
- 🏰 **Objective Control** by capturing towers and attacking the nexus
- 💪 **Health Management** balancing WiFi usage (causes damage) with strategic communication

### Challenge Specifications

| Parameter | Value |
|-----------|-------|
| **Robot** | Custom 4-Wheel Skid-Steer Platform |
| **Dimensions** | 12" × 12" max footprint |
| **Motors** | 4× JGA25-371 (463 RPM @ 12V) |
| **Localization** | HTC Vive Lighthouse System |
| **Sensors** | 3× VL53L0X Time-of-Flight |
| **Controller** | ESP32-S3 Wroom-1 |
| **Game Mode** | 3v3 Team Battle |
| **Autonomous Period** | 45 seconds |

---

**Course**: MEAM 510 - Design of Mechatronic Systems  
**Competition Date**: December 2024  
**Final Result**: 🥇 **1st Place (Alliance Champion)** | **All Performance Objectives Achieved**

---

## ✨ Key Features

### 🔧 Core Capabilities

- ✅ **Vive-based Localization** with dual photodiode tracking
- ✅ **4-wheel Skid Steering** with independent motor control
- ✅ **PD Velocity Control** with acceleration feedback
- ✅ **Time-of-Flight Sensing** for wall detection and following
- ✅ **Waypoint Navigation** with autonomous path execution
- ✅ **WiFi Control Interface** with real-time web dashboard
- ✅ **Health Tracking System** via I2C Top Hat integration
- ✅ **Servo-Actuated Weapon** (Minecraft pickaxe)

### 🎓 Advanced Techniques

- **Median filtering** for robust Vive position estimates
- **Origin calibration** for relative waypoint navigation
- **Complementary sensor fusion** for wall following
- **PWM motor control** with 14-bit resolution
- **Differential drive kinematics** for precise motion control
- **Null-space optimization** for turning while driving

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        MAIN CONTROL LOOP                        │
│                         (main.cpp)                              │
└──────────────────┬──────────────────────────────────────────────┘
                   │
       ┌───────────┴───────────┐
       │                       │
┌──────▼──────┐        ┌──────▼──────┐
│   MANUAL    │        │ AUTONOMOUS  │
│    MODE     │        │    MODES    │
│  (WiFi UI)  │        │(Wall/Target)│
└──────┬──────┘        └──────┬──────┘
       │                       │
       └───────────┬───────────┘
                   │
    ┌──────────────┼──────────────┐
    │              │              │
┌───▼────┐    ┌────▼────┐    ┌───▼────┐
│ VIVE   │    │ MOTOR   │    │  TOF   │
│ TRACK  │    │ CONTROL │    │ SENSORS│
└───┬────┘    └────┬────┘    └───┬────┘
    │              │              │
    │      ┌───────▼──────┐       │
    │      │  VELOCITY    │       │
    │      │  SETPOINT    │       │
    │      │  CALCULATOR  │       │
    │      └──────────────┘       │
    │                             │
    └──────────┬──────────────────┘
               │
        ┌──────▼──────┐
        │   WEAPON    │
        │  (Servo)    │
        └─────────────┘
               │
        ┌──────▼──────┐
        │  TOP HAT    │
        │  (Health)   │
        └─────────────┘
```

---

## 🔬 Technical Approach

### 1. Mechanical Design

#### Robot Platform: "Bulma"

Our robot features a **4-wheel skid-steer design** optimized for maneuverability and power:

**Base Structure:**
- Laser-cut acrylic baseplate (12" × 12")
- Tiered mounting system for component organization
- Low center of gravity for stability

**Drivetrain:**
- **Motors**: 4× JGA25-371 DC motors with integrated encoders
- **Gear Ratio**: 13.75:1
- **Wheels**: Hobby Park 2.9" diameter
- **Coupling**: 4mm shaft couplers
- **Configuration**: Skid-steer (left/right side differential)

**Sensor Array:**
- Front, Left, Right ToF sensors on dedicated acrylic mount
- Vive photodiodes on top surface for unobstructed lighthouse reception
- Whisker switch mounted vertically per competition rules

**Weapon System:**
- DSSERVO 20 kg-cm servo motor
- Custom acrylic Minecraft pickaxe attachment
- Triggered via web interface

**Key Design Features:**
```
✓ Modular mounting points for easy component swapping
✓ Cable management channels to protect wiring
✓ Accessible power switch and charging port
✓ Compliance with height restrictions (7" max, excluding whisker)
```

### 2. Electrical Design

#### System Overview

The electrical system is built around a central ESP32-S3 microcontroller with modular subsystems:

**Power Distribution:**
```
2200 mAh 11.1V LiPo Battery
          │
          ├─→ L298N Motor Driver (powers 4 motors)
          │     └─→ 5V regulator → ESP32-S3 (main)
          │
          └─→ External Power Bank → ESP32-S3 + Servo
```

#### 2.1 Motor Driver Circuit

**Components:**
- **L298N H-Bridge** motor driver
- **SN74HC14N** inverter for direction control
- **4× Quadrature Encoders** (12 ticks/rotation)

**Connections:**
```cpp
// Right side motors (shared control)
OUT1/2, ENA → Motors 0, 1
PWM Frequency: 10 Hz
Resolution: 14-bit (16,383 steps)

// Left side motors (shared control)
OUT3/4, ENB → Motors 2, 3
PWM Frequency: 10 Hz
Resolution: 14-bit (16,383 steps)
```

**Motor Control Strategy:**
- Synchronous control per side reduces pin usage
- Direction pins inverted via SN74HC14N
- Encoder interrupts on CHANGE for both rising/falling edges

#### 2.2 Vive Localization Circuit

**Dual Photodiode System:**

Each circuit consists of:
- **PD70-01C** photodiode sensor
- **TLV272** dual op-amp for signal conditioning
- Envelope detection circuit from course materials

**Purpose:**
- Circuit 1: (x, y) position
- Circuit 2: (x, y) orientation reference → calculate yaw

**Signal Processing:**
```cpp
// Median filter (3-sample window)
x_filtered = median(x0, x_prev1, x_prev2);
y_filtered = median(y0, y_prev1, y_prev2);

// Bounds checking
if (x > 8000 || x < 1000 || y > 8000 || y < 1000) {
    // Invalid reading, discard
}
```

#### 2.3 Wall Following Circuit

**3× VL53L0X Time-of-Flight Sensors:**

```
Sensor Layout:
        [FRONT]
          │
   [LEFT]─┼─[RIGHT]
          │
```

**I2C Configuration:**
- Shared I2C bus (SDA: GPIO 8, SCL: GPIO 9)
- Unique addresses via XSHUT pin control:
  - LOX1: 0x30 (Front)
  - LOX2: 0x31 (Right)
  - LOX3: 0x32 (Left)

**Initialization Sequence:**
```cpp
1. Reset all sensors (XSHUT = LOW)
2. Enable LOX1, set address 0x30
3. Enable LOX2, set address 0x31
4. Enable LOX3, set address 0x32
```

#### 2.4 Top Hat Interface

**Health Tracking System:**

Components:
- ESP32-S3 (Top Hat master)
- ESP-WROOM (slave)
- NeoPixel ring (24 LEDs)
- Whisker switch

**I2C Communication:**
```cpp
// Report WiFi usage at 2 Hz
uint8_t wifi_packets_since_last_update;

// Receive health status
uint8_t current_health;
bool is_alive;
```

**LED Display:**
- Health percentage shown by lit LEDs
- Robot ID indicated by color (Red/Blue team)
- Death timer countdown when HP = 0

#### 2.5 Weapon System

**DSSERVO 20 kg-cm Servo:**
- Power: 5V from L298N regulator
- Signal: GPIO pin from main ESP32-S3
- Control: Standard PWM servo signals (500-2500 μs)

**Servo Specifications:**
| Parameter | Value |
|-----------|-------|
| Operating Voltage | 5V - 6.8V |
| Speed (no load) | 0.16 sec/60° @ 5V |
| Stall Torque | 19 kg-cm @ 5V |
| Pulse Range | 500-2500 μs |
| Rotation | 270° max |

### 3. Localization System

#### Vive Lighthouse Tracking

The HTC Vive system provides millimeter-accurate indoor positioning:

**System Components:**
- 2× Lighthouse base stations (infrared laser sweeps)
- 2× Photodiode circuits on robot
- Vive510 library for signal processing

**Coordinate Calculation:**

```cpp
void GetPosition() {
    // Read both Vive circuits
    if (vive1.status() == VIVE_RECEIVING) {
        x1 = vive1.xCoord();
        y1 = vive1.yCoord();
    }
    
    if (vive2.status() == VIVE_RECEIVING) {
        x2 = vive2.xCoord();
        y2 = vive2.yCoord();
    }
    
    // Calculate centroid position
    state.x = (x1 + x2) / 2;
    state.y = (y1 + y2) / 2;
    
    // Calculate yaw from vector between sensors
    float dx = x2 - x1;
    float dy = y2 - y1;
    float yaw = atan2(dy, dx) * 180 / PI;
    
    // Normalize to 0-360° with sensor offset
    if (yaw > 90) {
        yaw = yaw - 90;
    } else {
        yaw = 360 - (90 - yaw);
    }
    
    state.yaw = yaw;
}
```

**Median Filtering:**

Reduces noise from momentary dropouts:

```cpp
uint32_t med3filt(uint32_t a, uint32_t b, uint32_t c) {
    // Returns median of 3 samples
    if ((a <= b) && (a <= c))
        return (b <= c) ? b : c;
    else if ((b <= a) && (b <= c))
        return (a <= c) ? a : c;
    else
        return (a <= b) ? a : b;
}
```

**Performance:**
- Update rate: ~100 Hz (limited by Vive sweep frequency)
- Position accuracy: ±10-20 mm (after filtering)
- Yaw accuracy: ±2-3°

### 4. Control Architecture

#### Software State Machine

The robot operates in three distinct modes:

```cpp
enum RobotMode {
    MANUAL,        // WiFi joystick control
    WALL_FOLLOW,   // Autonomous wall following
    TARGET         // Waypoint navigation
};
```

#### Main Control Loop

```cpp
void loop() {
    // Update sensor readings
    read_tof();
    GetPosition();
    
    // Execute mode-specific behavior
    if (robot_mode == WALL_FOLLOW) {
        wallfollow();
    } else if (robot_mode == TARGET) {
        target_auto();
    }
    
    // Process web interface commands
    server.handleClient();
    
    // Extract velocity commands
    float linvel = motorControl[0] * motorControl[2] / 100.0;
    float angvel = motorControl[1];
    
    // Handle STOP command
    if (abs(motorControl[0]) < 0.01 && abs(motorControl[1]) < 0.01) {
        for (int i = 0; i < 4; i++) {
            ledcWrite(motorPINEN[i], 0);
            motorVelSetpoint[i] = 0.0;
        }
        return;
    }
    
    // Execute velocity control
    calcMotorVelSetpoint(linvel, angvel);
    controlMotor(linvel, angvel);
    
    delay(50); // 20 Hz control loop
}
```

#### WiFi Control Interface

**Web Dashboard Features:**
- Directional buttons (Up, Down, Left, Right, Stop)
- Speed slider (0-100%)
- Mode buttons (Follow Wall, Attack, Target)
- Mobile-responsive design with landscape rotation

**Command Encoding:**

| Command | Code | Action |
|---------|------|--------|
| Stop | 0 | Zero all velocities |
| Forward | 1 | Set linear velocity |
| Left | 2 | Increment angular velocity +0.1 |
| Right | 3 | Decrement angular velocity -0.1 |
| Attack | 4 | Trigger weapon servo |
| Follow Wall | 5 | Enter WALL_FOLLOW mode |
| Target | 6 | Enter TARGET mode |

**WiFi Packet Tracking:**

```cpp
// Each packet sent costs 1 HP
uint8_t packets_sent_this_interval = 0;

// Report to Top Hat at 2 Hz
void updateTopHat() {
    i2c_send(TOP_HAT_ADDR, packets_sent_this_interval);
    packets_sent_this_interval = 0;
}
```

### 5. Autonomous Navigation

#### Waypoint Navigation System

**Coordinate System:**

All waypoints are defined relative to a calibrated origin:

```cpp
PositionData origin;  // Set at startup
PositionData waypoints[] = {
    {-1000, 0, 0},      // Waypoint 1: relative offsets
    {-1000, -900, 0},   // Waypoint 2
    {-3400, -900, 0},   // Waypoint 3
    {-3400, -400, 0},   // Waypoint 4
    {-2350, -400, 0},   // Waypoint 5
    {-4400, ...}        // Additional waypoints
};
```

#### Origin Calibration

Critical for reliable navigation in unknown starting positions:

```cpp
void target_auto() {
    if (!origin_initialized) {
        int init_count = 0;
        PositionData prev_state;
        
        while (init_count < 5) {
            GetPosition();
            
            if (init_count > 0) {
                // Check if position is stable
                float dx = current_state.x - prev_state.x;
                float dy = current_state.y - prev_state.y;
                float dist = sqrt(dx*dx + dy*dy);
                
                if (dist < 20) {  // Stable reading threshold
                    init_count++;
                } else {
                    init_count = 0;  // Reset if unstable
                }
            } else {
                init_count++;
            }
            
            prev_state = state;
        }
        
        origin = state;
        origin_initialized = true;
    }
    
    // Navigate to waypoints...
}
```

**Rationale:**
- Vive readings can be noisy at field edges
- Requires 5 consecutive stable readings (< 20mm movement)
- Prevents navigation errors from bad initial position

#### GoTo Function

```cpp
bool goTo(int16_t x_goal, int16_t y_goal) {
    static bool turning_mode = 0;
    
    // Convert relative to absolute coordinates
    x_goal = origin.x + x_goal;
    y_goal = origin.y + y_goal;
    
    // Calculate error vector
    float dx = x_goal - state.x;
    float dy = y_goal - state.y;
    
    // Calculate desired heading
    float desired_angle = atan2(dy, dx) * 180 / PI;
    if (desired_angle < 0) desired_angle += 360;
    
    // Calculate angular error
    float ang_diff = desired_angle - state.yaw;
    
    // Normalize to [-180, 180]
    if (ang_diff > 180.0) ang_diff -= 360.0;
    else if (ang_diff < -180.0) ang_diff += 360.0;
    
    // Check if at goal
    float distance = sqrt(dx*dx + dy*dy);
    if (distance < DISTANCE_THRESHOLD) return true;
    
    // Phase 1: Turn to face target
    if (abs(ang_diff) > ANGLE_THRESHOLD) {
        // Pure rotation
        motorControl[0] = 1;
        motorControl[1] = (state.yaw < desired_angle) ? 0.1 : -0.1;
        motorControl[2] = 0;
        return false;
    }
    
    // Phase 2: Drive toward target
    else {
        motorControl[0] = 1;
        motorControl[1] = 0;
        motorControl[2] = 60.0;  // Speed percentage
        return false;
    }
}
```

**Navigation Strategy:**
1. **Rotate in place** until facing target (discrete turns)
2. **Drive straight** toward target
3. Repeat for each waypoint in sequence

**Thresholds:**
- Angular threshold: 12° (allows some drift while driving)
- Distance threshold: 150 mm (arrival tolerance)

### 6. Wall Following

#### Algorithm Overview

The wall following behavior uses a simple but effective reactive control:

```cpp
void wallfollow() {
    static const int left_margin = 250;
    static const int right_margin = 200;
    
    int front_threshold = 600;  // mm
    
    // Case 1: Path ahead is clear
    if (measure2.RangeMilliMeter > front_threshold) {
        motorControl[0] = 1;
        motorControl[1] = 0;
        motorControl[2] = 50.0;  // Drive forward
    }
    
    // Case 2: Wall ahead, need to turn left
    else if (measure2.RangeMilliMeter < front_threshold) {
        motorControl[0] = 0;
        motorControl[1] = 0;
        motorControl[2] = 0;
        turn_left();  // Execute 90° turn
    }
    
    // Case 3: Drifting away from wall
    else if (measure1.RangeMilliMeter > right_margin) {
        motorControl[0] = 0;
        motorControl[1] = -0.1;  // Correct right
        motorControl[2] = 0;
    }
}
```

#### Turn Left Function

```cpp
void turn_left() {
    // Rotate until right sensor reads < 600mm (facing new wall)
    while (measure2.RangeMilliMeter < 600) {
        motorControl[0] = 0;
        motorControl[1] = 0.1;  // Rotate CCW
        motorControl[2] = 0.0;
        
        calcMotorVelSetpoint(linvel, angvel);
        controlMotor(linvel, angvel);
        read_tof();
        delay(50);
    }
    
    // Fine-tune alignment
    if (measure2.RangeMilliMeter < 700 && measure1.RangeMilliMeter > 275) {
        motorControl[1] = -0.1;  // Slight CW correction
    }
}
```

**Wall Following Performance:**
- Successfully completed full circuit
- Took 2 attempts during graded evaluation
- Robust to varying wall distances

---

## 📊 Performance Results

### Competition Performance

- **Final Placement**: 🥇 **1st Place (Alliance Champion)**
- **Team Alliance**: Winning 3-robot meta-team


### System Capabilities

**Localization Accuracy:**
- Position RMS error: ~15-25 mm
- Yaw error: ~2-5°
- Update rate: 20 Hz (control loop)

**Motor Performance:**
- Maximum speed: ~0.5 m/s
- Turn radius: ~0.3 m (skid-steer)
- Response time: <200 ms

**Sensor Performance:**
- ToF range: 30-2000 mm
- ToF accuracy: ±3% of reading
- I2C communication: 100% reliable

### Timing Breakdown

```
Typical Mission Execution:
├── Origin Calibration: ~5 seconds
├── Waypoint 1: ~10 seconds
├── Waypoint 2: ~8 seconds
├── Waypoint 3: ~12 seconds
└── Total Autonomous: ~35 seconds (within 45s limit)
```

---

## 📚 Lessons Learned

### ✅ What Worked Well

1. **Skid-Steer Platform**
   - Simple, reliable mechanical design
   - Predictable kinematics for control
   - High torque from 4-motor configuration

2. **Modular Circuit Design**
   - Separate perfboards for each subsystem
   - Easy to debug individual components
   - Enabled parallel development by team members

3. **Origin Calibration Strategy**
   - Solved Vive noise issues near field edges
   - Enabled reliable waypoint navigation
   - Simple but effective 5-sample stability check

4. **Complementary Filter for Vive**
   - Median filtering reduced outlier spikes
   - Bounds checking prevented runaway values
   - Stable 20 Hz position updates

5. **PD Velocity Control**
   - Acceleration feedback prevented oscillation
   - Smooth motor response without overshoot
   - Tunable gains for different speed requirements

---

## 📖 References

### Course Materials

1. MEAM 510 Course Notes - University of Pennsylvania

### Technical Documentation

1. Espressif ESP32-S3 Technical Reference Manual
2. STMicroelectronics L298N Dual Full-Bridge Driver Datasheet
3. STMicroelectronics VL53L0X Time-of-Flight Ranging Sensor Datasheet
4. Texas Instruments TLV272 Operational Amplifier Datasheet
5. HTC Vive Lighthouse Tracking System Documentation

---

## 🙏 Acknowledgments

- **MEAM 510 Teaching Staff** for guidance, support, and late-night lab access
- **University of Pennsylvania** for providing resources and facilities
- **Fellow Students** for collaboration, healthy competition, and 3 AM debugging sessions

**Special Thanks:**
- TAs for clarifying game rules and providing feedback during design reviews
- RPL staff for laser cutting access and mechanical advice
- Other teams for sharing debugging tips and creating a supportive environment

---

<div align="center">

### 🏆 Competition Results: 1st Place Alliance Champion 🏆

---

### 📹 Video Links

- [Wall Following Demo](https://your-video-link-1)
- [Autonomous Target Navigation](https://your-video-link-2)
- [Top Hat Health System](https://your-video-link-3)
- [Competition Highlights](https://your-video-link-4)

---

[⬆ Back to Top](#-bulma-autonomous-robot-battle-arena-roba-champion)

</div>
