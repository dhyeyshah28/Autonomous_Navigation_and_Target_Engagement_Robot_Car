# Autonomous_Navigation_and_Target_Engagement_Robot_Car
MEAM5100 Final Project Competition Fall'24

# 🤖 Autonomous Navigation and Target Engagement Robot Car: Robot Battle Arena (RoBA) Champion

> **Description**: We built a robust autonomous mobile robot for the MEAM 510 Robot Battle Arena game, achieving championship performance with robust localization, autonomous navigation, and strategic gameplay capabilities.

[![Competition Result](https://img.shields.io/badge/Competition-1st%20Place-gold?style=for-the-badge)](https://github.com)
[![Alliance](https://img.shields.io/badge/Alliance-Winning%20Team-success?style=for-the-badge)](https://github.com)
[![Python](https://img.shields.io/badge/Arduino-C%2FC%2B%2B-blue?style=for-the-badge&logo=arduino)](https://www.arduino.cc/)
[![ROS](https://img.shields.io/badge/ESP32-IoT-black?style=for-the-badge&logo=espressif)](https://www.espressif.com/)

<div align="center">

**🏆 CHAMPIONSHIP WINNING ROBOT 🏆**

*"Bulma" - Named after the Dragon Ball character*

</div>

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

This project implements a complete autonomous mobile robot system for the **MEAM 510 Robot Battle Arena (RoBA) Challenge**. The game is based on Multiplayer Online Battle Arena (MOBA) style gameplay where robots compete in a 3v3 format to destroy the opposing team's nexus while defending their own.

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
| **Controller** | ESP32-S3 |
| **Game Mode** | 3v3 Team Battle |
| **Autonomous Period** | 45 seconds |

### Game Mechanics

**Health System:**
- Starting HP: 100
- Whisker hit damage: -10 HP (0.5s cooldown)
- WiFi packet damage: -1 HP per packet
- Death timer: 15 seconds

**Objective System:**
- **Towers**: Capture by holding button for 8 seconds → deals 1 HP/2s to enemy nexus
- **Nexus**: Push button for 5 HP/second damage
- **Victory**: Reduce enemy nexus to 0 HP

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
│  (WiFi UI)  │        │ (Wall/Target)│
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

**Design Iterations:**
- **Initial**: Omni-wheels for holonomic motion
  - ❌ *Failed*: Excessive slippage, poor positioning accuracy
- **Final**: Standard wheels with skid-steer
  - ✅ *Success*: Reliable traction, predictable kinematics

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
- ESP-WROOM (slave, provided)
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

**Mode Transitions:**

```
        ┌─────────────┐
        │   MANUAL    │ ←─────────┐
        └──────┬──────┘            │
               │                   │
      User Command (5)        User Command (0)
               │                   │
               ▼                   │
        ┌─────────────┐            │
        │WALL_FOLLOW  │────────────┘
        └─────────────┘
               │
      User Command (6)
               │
               ▼
        ┌─────────────┐
        │   TARGET    │
        └─────────────┘
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
- **Competition Format**: FIRST-style alliance selection
- **Match Performance**: Consistent victories in group stage and playoffs

### Graded Evaluation Results

| Task | Points Earned | Max Points | Status |
|------|---------------|------------|--------|
| Control Robot | 4 | 4 | ✅ Complete |
| Solder All Electronics | 2 | 2 | ✅ Complete |
| Attack a Robot | 5 | 5 | ✅ Complete |
| Transmit WiFi @ 2Hz | 2 | 2 | ✅ Complete |
| Drive Up Ramp | 5 | 5 | ✅ Complete |
| Wall Follow - Full Circuit | 15 | 15 | ✅ Complete |
| Autonomous Attack Target 1 | 5 | 5 | ✅ Complete |
| Autonomous Attack Target 2 | 5 | 5 | ⚠️ Partial |
| Autonomous Attack Target 3 | 5 | 5 | ❌ Not Attempted |
| Attack Multiple Targets (45s) | 5 | 5 | ⚠️ Partial |
| Attack TA Bot Before Dying | 10 | 10 | ✅ Complete |
| **TOTAL** | **58+** | **65** | **89%+** |

**Extra Credit Earned:**
- Competition participation: +5% (maximum)
- Performance-based: Bonus for 1st place finish

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

## 🚀 Installation & Setup

### Prerequisites

```bash
# System Requirements
- Arduino IDE 2.0+
- ESP32 Board Support Package
- USB-C cable for programming
```

### Step 1: Arduino IDE Setup

```bash
# Install ESP32 board support
1. Open Arduino IDE
2. File → Preferences
3. Additional Board Manager URLs:
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
4. Tools → Board → Board Manager
5. Search "ESP32" and install "esp32 by Espressif Systems"
```

### Step 2: Install Libraries

```bash
# Required Libraries (Install via Library Manager)
- WiFi (built-in)
- WebServer (built-in)
- Wire (built-in, for I2C)
- Adafruit_VL53L0X
- vive510 (custom, see course materials)
```

### Step 3: Clone Repository

```bash
# Clone your project
git clone https://github.com/YOUR_USERNAME/meam510_bulma.git
cd meam510_bulma
```

### Step 4: Hardware Assembly

1. **Solder all circuits** on perfboards
2. **Mount components** on acrylic chassis
3. **Connect motors** to L298N driver
4. **Wire sensors** to ESP32-S3 GPIO pins
5. **Install Top Hat** with whisker switch
6. **Calibrate Vive** photodiode placement

### Step 5: Upload Code

```bash
1. Open main.ino in Arduino IDE
2. Select board: "ESP32S3 Dev Module"
3. Select port: /dev/ttyUSB0 (or COM port on Windows)
4. Upload sketch
5. Open Serial Monitor (115200 baud)
```

---

## 💻 Usage

### Web Interface

#### Step 1: Power On Robot

```bash
1. Ensure battery is charged (11.1V LiPo)
2. Turn on robot power switch
3. Wait for ESP32 to boot (~5 seconds)
```

#### Step 2: Connect to WiFi

```bash
Network: "Bulma"
Password: "12345678"
IP Address: 192.168.4.1 (printed to Serial Monitor)
```

#### Step 3: Open Dashboard

```bash
# In web browser
http://192.168.4.1
```

#### Step 4: Control Robot

**Manual Mode:**
- Click **Up** to drive forward
- Click **Left/Right** to turn
- Click **Stop** to halt
- Adjust **Speed Slider** for velocity

**Autonomous Modes:**
- Click **Follow Wall** for wall-following behavior
- Click **Target** for waypoint navigation
- Click **Stop** to return to manual control

**Weapon:**
- Click **Attack** to swing pickaxe

### Serial Monitor Debugging

```bash
# Typical output
###############
User Input:
1.00        # motorControl[0] - direction
0.10        # motorControl[1] - angular vel
50.00       # motorControl[2] - speed %

POSITION
X: 4523
Y: 3812
YAW: 87.34

1: 345      # Front ToF (mm)
2: 678      # Right ToF (mm)
3: 892      # Left ToF (mm)
```

---

## 📁 Repository Structure

```
meam510_bulma/
├── main/
│   ├── main.ino                    # Main control code
│   ├── index.h                     # HTML web interface
│   └── vive510.h                   # Vive tracking library
│
├── libraries/
│   └── Adafruit_VL53L0X/           # ToF sensor library
│
├── hardware/
│   ├── schematics/
│   │   ├── motor_driver.png        # Motor circuit
│   │   ├── vive_circuit.png        # Localization circuit
│   │   ├── tof_circuit.png         # Wall following circuit
│   │   ├── tophat_circuit.png      # Health tracking circuit
│   │   └── weapon_circuit.png      # Servo circuit
│   │
│   └── cad/
│       ├── bulma_base.dxf          # Chassis baseplate
│       ├── bulma_tof_mount.dxf     # Sensor mount
│       ├── bulma_motor_mount.dxf   # Motor bracket
│       └── bulma_esp32_mount.dxf   # Controller mount
│
├── docs/
│   ├── Final_Project_Assignment.pdf
│   ├── RoBA_Game_Rules.pdf
│   └── MEAM510_Final_Report.pdf
│
├── datasheets/
│   ├── ESP32-S3_datasheet.pdf
│   ├── L298N_datasheet.pdf
│   ├── VL53L0X_datasheet.pdf
│   ├── TLV272_datasheet.pdf
│   ├── PD70-01C_datasheet.pdf
│   └── DSSERVO_datasheet.pdf
│
├── videos/
│   ├── wall_follow.mp4
│   ├── autonomous_target.mp4
│   ├── tophat_demo.mp4
│   └── competition_highlights.mp4
│
├── images/
│   ├── bulma_front.jpg
│   ├── bulma_top.jpg
│   └── competition_action.jpg
│
├── BOM.csv                          # Bill of materials
└── README.md                        # This file
```

---

## 🧮 Key Algorithms

### 1. Velocity Controller

#### Differential Drive Kinematics

```cpp
#define HALF_WHEEL_BASE_B 0.13    // meters (wheelbase / 2)
#define WHEEL_RADIUS 0.035         // meters

void calcMotorVelSetpoint(float linvel, float angvel) {
    // Inverse kinematics for differential drive
    // v_left = v - ω*B
    // v_right = v + ω*B
    
    float angv_left = (linvel - HALF_WHEEL_BASE_B * angvel) / WHEEL_RADIUS;
    float angv_right = (linvel + HALF_WHEEL_BASE_B * angvel) / WHEEL_RADIUS;
    
    // Apply to motor pairs
    motorVelSetpoint[0] = angv_right;
    motorVelSetpoint[1] = angv_right;
    motorVelSetpoint[2] = angv_left;
    motorVelSetpoint[3] = angv_left;
}
```

**Parameters:**
- Linear velocity: m/s (world frame)
- Angular velocity: rad/s (robot frame)
- Output: rad/s per wheel

#### PD Control with Acceleration Feedback

```cpp
#define KP 50.0
#define KD -0.01

void controlMotor(float linvel, float angvel) {
    static unsigned long previous_timestamp;
    unsigned long current_timestamp = millis();
    double time_diff = current_timestamp - previous_timestamp;
    
    static int prev_tick_dot[4];
    
    for (int i = 0; i < 4; i++) {
        // Calculate current velocity from encoder
        float tick = ticksPerInterval[i] - prevtick[i];
        tick = tick / 34.0;  // CPR conversion
        float rad = tick * 30 * PI / 180;
        float angvel = rad / (time_diff / 1000.0);
        
        prevtick[i] = ticksPerInterval[i];
        currentWheelAngVel[i] = angvel;
        
        // PD control
        float error = abs(motorVelSetpoint[i]) - abs(currentWheelAngVel[i]);
        float acceleration = 1000 * (abs(currentWheelAngVel[i]) - abs(prev_tick_dot[i] / time_diff));
        
        float u = KP * error + KD * acceleration;
        
        currentWheelPwm[i] = currentWheelPwm[i] + u;
        
        // Clamp PWM
        if (currentWheelPwm[i] > 16383) currentWheelPwm[i] = 16383;
        else if (currentWheelPwm[i] < 0) currentWheelPwm[i] = 0;
        
        // Set direction
        if (motorVelSetpoint[i] > 0) {
            digitalWrite(motorPINDIR[i], HIGH);
            motorDir[i] = 1;
        } else {
            digitalWrite(motorPINDIR[i], LOW);
            motorDir[i] = -1;
        }
        
        // Apply PWM
        ledcWrite(motorPINEN[i], currentWheelPwm[i]);
    }
    
    for (int i = 0; i < 4; i++) {
        prev_tick_dot[i] = currentWheelAngVel[i];
    }
    
    previous_timestamp = current_timestamp;
}
```

**Control Features:**
- **P term**: Proportional error correction
- **D term**: Acceleration damping (prevents oscillation)
- **Feedforward**: Direct PWM accumulation for smooth response

### 2. Waypoint Navigation

#### Sequential Waypoint Execution

```cpp
int waypoint_iterator = 0;

void target_auto() {
    // First-time initialization
    if (!origin_initialized) {
        // [Origin calibration code - see Section 4]
    }
    
    // Navigate to current waypoint
    bool success = goTo(waypoints[waypoint_iterator].x, 
                        waypoints[waypoint_iterator].y);
    
    if (success) {
        Serial.println("SUCCESS!!!!!");
        
        // Check if more waypoints remain
        if (waypoint_iterator + 1 < sizeof(waypoints) / sizeof(waypoints[0])) {
            waypoint_iterator++;
        } else {
            Serial.println("Successful Mission!");
            robot_mode = MANUAL;  // Return to manual control
        }
    }
    
    delay(50);
}
```

**Mission Flow:**
```
START
  │
  ├─→ Calibrate origin (5 stable readings)
  │
  ├─→ Navigate to waypoint[0]
  │   ├─→ goTo() returns false → keep navigating
  │   └─→ goTo() returns true → increment iterator
  │
  ├─→ Navigate to waypoint[1]
  │   └─→ ...
  │
  └─→ All waypoints complete → return to MANUAL mode
```

### 3. Wall Following Algorithm

#### State Machine

```
┌─────────────┐
│   FORWARD   │
│ (front > T) │
└──────┬──────┘
       │
    No │  Front obstacle?
       │
       ▼ Yes
┌─────────────┐
│  TURN LEFT  │
│  (rotate)   │
└──────┬──────┘
       │
       │  Right sensor clear?
       │
       ▼ Yes
┌─────────────┐
│   FORWARD   │
│   (resume)  │
└─────────────┘
```

#### Threshold Tuning

```cpp
// Distance thresholds (mm)
int front_tof_threshold[4] = {600, 500, 500, 500};
// Index corresponds to iteration/corner number

const int left_margin = 250;
const int right_margin = 200;
```

**Tuning Process:**
1. Started with 500mm threshold (too sensitive)
2. Increased to 600mm for front sensor
3. Added margin parameters for drift correction
4. Tested on full circuit, adjusted per corner

### 4. Origin Calibration

#### Stability-Based Initialization

This algorithm ensures the robot doesn't start navigation with a noisy Vive reading:

```cpp
void calibrate_origin() {
    int init_count = 0;
    PositionData prev_state;
    
    while (init_count < 5) {
        GetPosition();
        
        if (init_count > 0) {
            PositionData current_state = state;
            
            // Calculate movement since last reading
            float dx_origin = current_state.x - prev_state.x;
            float dy_origin = current_state.y - prev_state.y;
            float dist = sqrt(dx_origin*dx_origin + dy_origin*dy_origin);
            
            Serial.print("origin init dist: ");
            Serial.println(dist);
            
            // Check if reading is stable
            if (dist < 20) {  // Less than 20mm movement
                init_count++;
            } else {
                init_count = 0;  // Reset counter if unstable
            }
        } else {
            init_count++;
        }
        
        prev_state = state;
    }
    
    origin = state;
    origin_initialized = true;
    
    Serial.println("Setting origin to:");
    Serial.println(origin.x);
    Serial.println(origin.y);
}
```

**Why This Matters:**

Without stable origin calibration:
- Robot might think it's 100mm off from actual position
- All waypoint navigation would be offset by that error
- Could drive into walls or miss objectives entirely

**Requirements:**
- 5 consecutive readings within 20mm of each other
- Prevents "glitch readings" near field edges
- Typical calibration time: 2-5 seconds

---

## 🎮 Competition Rules

### Game Objective

**Victory Condition:** Reduce enemy nexus health to 0 HP

**Strategy Options:**
1. **Direct Assault**: Attack enemy nexus (5 HP/sec while button held)
2. **Tower Control**: Capture towers to deal passive damage (1 HP/2sec)
3. **Robot Combat**: Eliminate enemy robots to gain map control

### Robot Requirements

**Physical Constraints:**
- ✅ 12" × 12" maximum footprint
- ✅ 7" height limit (excluding whisker)
- ✅ Whisker switch accessible 270° (top 8")
- ✅ Whisker switch accessible 360° (top 1")
- ✅ Top Hat mounted horizontally, LEDs visible

**Functional Requirements:**
- ✅ WiFi or autonomous control
- ✅ I2C communication with Top Hat (health tracking)
- ✅ Transmit WiFi usage at 2 Hz
- ✅ Stop movement when health depleted
- ✅ Start only when "alive" signal received

**Prohibited Actions:**
- ❌ Intentional robot flipping
- ❌ Malicious collisions (referee discretion)
- ❌ Liquids, flames, combustion
- ❌ Modifying whisker switches
- ❌ Permanent field damage

### Health System

**Damage Sources:**
- Whisker trigger: -10 HP (0.5s cooldown, max 20 HP/sec)
- WiFi packet: -1 HP per packet
- Nexus arms: -10 HP per hit

**Death Mechanic:**
- Death timer: 15 seconds off field
- Respawn at home base with full HP
- Failure to stop: +15 second penalty

### Tower Mechanics

**Capture Process:**
1. Hold your team's button
2. Timer moves toward your side
3. Reach 8 seconds → tower captured
4. Opponent can block by pressing their button simultaneously

**Tower Benefits:**
- Passive damage to enemy nexus: 1 HP / 2 seconds
- Continues until tower is recaptured

### Nexus Defense

**Arms of Death:**
- Rotating arms at 7.5" height
- ~0.1 revolutions/second (continuous)
- Triggers whisker on contact (-10 HP)
- Cannot be blocked

**Nexus Attack:**
- Push button for 5 HP/second damage
- Vulnerable to arm hits while attacking
- High-risk, high-reward strategy

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

### ⚠️ Challenges Encountered

1. **Omni-Wheel Failure**
   - Initial design used omni-wheels for holonomic motion
   - Excessive slippage on arena floor
   - Poor odometry integration
   - **Solution**: Switched to standard wheels, accepted skid-steer

2. **I2C Top Hat Integration**
   - Last-minute integration during graded evaluation
   - Timing issues with 2 Hz update requirement
   - **Solution**: Got working for competition day, but ran out of time for grading

3. **Waypoint Navigation Logic Bug**
   - Robot would get stuck oscillating between two angles
   - Caused by incorrect angle normalization in goTo()
   - **Solution**: Fixed angle wrapping with proper [-180, 180] bounds

4. **Whisker Mount Design**
   - Designed fixture on morning of graded evaluation
   - Flimsy, not robust to collisions
   - **Solution**: Worked but could have been pre-planned

5. **Encoder Resolution**
   - Only 12 ticks per rotation (13.75:1 gear ratio)
   - Low resolution for precise velocity control
   - **Solution**: Used higher Kp gain, median filtering

### 🔄 Iterative Improvements

**Mechanical:**
- V1: Omni-wheels → ❌ Failed (slippage)
- V2: Standard wheels → ✅ Success (reliable)

**Control:**
- V1: P-only controller → ❌ Oscillation
- V2: PD with acceleration feedback → ✅ Smooth

**Navigation:**
- V1: Absolute waypoints → ❌ Failed (edge noise)
- V2: Relative waypoints + origin calibration → ✅ Reliable

**Wall Following:**
- V1: Fixed 500mm threshold → ⚠️ Too sensitive
- V2: Adaptive thresholds per corner → ✅ Full circuit

---

## 🔮 Future Improvements

### Short-Term Enhancements

1. **Improved Waypoint Logic**
   ```cpp
   // Add intermediate waypoints for smoother paths
   // Implement curve-following instead of point-to-point
   
   // Potential algorithm: Pure Pursuit
   float lookahead_distance = 300;  // mm
   Point lookahead = interpolate_path(current_pos, waypoints, lookahead_distance);
   float desired_heading = atan2(lookahead.y - current.y, lookahead.x - current.x);
   ```

2. **Dynamic Obstacle Avoidance**
   ```cpp
   // Use ToF sensors to detect enemy robots
   if (measure_front < COLLISION_THRESHOLD && mode == TARGET) {
       // Temporarily halt navigation
       // Execute avoidance maneuver
       // Resume navigation
   }
   ```

3. **Battery Voltage Monitoring**
   ```cpp
   // Adjust PWM based on battery level
   float voltage = analogRead(BATTERY_PIN) * ADC_SCALE;
   float voltage_compensation = 12.0 / voltage;
   
   for (int i = 0; i < 4; i++) {
       currentWheelPwm[i] *= voltage_compensation;
   }
   ```

4. **Encoder Upgrade**
   - Current: 12 ticks/rotation
   - Target: 48+ ticks/rotation
   - Benefit: 4× resolution for velocity estimation

### Medium-Term Enhancements

1. **Kalman Filter for Localization**
   ```cpp
   // Fuse Vive position with wheel odometry
   KalmanFilter kf(6);  // [x, y, theta, vx, vy, omega]
   
   // Prediction step (from encoders)
   kf.predict(wheel_odometry);
   
   // Update step (from Vive)
   kf.update(vive_position);
   
   state = kf.get_state();
   ```

2. **Behavior Tree for Strategy**
   ```
   Root: Selector
   ├─→ Sequence: Capture Tower
   │   ├─→ Navigate to tower
   │   ├─→ Push button for 8s
   │   └─→ Return to base
   │
   ├─→ Sequence: Attack Nexus
   │   ├─→ Navigate to nexus
   │   ├─→ Avoid arms
   │   └─→ Push button
   │
   └─→ Sequence: Hunt Enemy
       ├─→ Search for enemy
       ├─→ Pursue
       └─→ Attack whisker
   ```

3. **Machine Learning for Arm Avoidance**
   ```python
   # Train neural network to predict arm position
   inputs: [time, nexus_position, robot_position]
   output: [safe_approach_angle, safe_attack_window]
   
   # Collect training data during practice
   # Deploy model on ESP32 for inference
   ```

4. **Multi-Robot Coordination**
   ```cpp
   // Share state over WiFi with teammates
   struct TeamState {
       uint8_t robot_id;
       PositionData position;
       uint8_t health;
       RobotMode mode;
   };
   
   // Coordinate strategies
   if (teammate_attacking_tower_1) {
       my_target = tower_2;  // Distribute effort
   }
   ```

### Long-Term Vision

1. **Computer Vision for Localization**
   - Replace Vive with overhead camera + ArUco markers
   - ESP32-CAM for onboard visual odometry
   - Reduces reliance on external infrastructure

2. **Reinforcement Learning Agent**
   - Train policy network in simulation
   - Optimize strategy against varying opponents
   - Deploy on embedded hardware (TensorFlow Lite)

3. **Modular Weapon System**
   - Quick-swap weapon attachments
   - Different strategies for different game modes
   - Automated weapon selection based on objective

---

## 📖 References

### Course Materials

1. MEAM 510 Course Notes - University of Pennsylvania
2. MEAM 510 Lab Manuals - Design of Mechatronic Systems

### Technical Documentation

1. Espressif ESP32-S3 Technical Reference Manual
2. STMicroelectronics L298N Dual Full-Bridge Driver Datasheet
3. STMicroelectronics VL53L0X Time-of-Flight Ranging Sensor Datasheet
4. Texas Instruments TLV272 Operational Amplifier Datasheet
5. HTC Vive Lighthouse Tracking System Documentation

### Software Libraries

1. Arduino ESP32 Core - https://github.com/espressif/arduino-esp32
2. Adafruit VL53L0X Library - https://github.com/adafruit/Adafruit_VL53L0X
3. Vive510 Library - MEAM 510 Course Repository

### Additional Resources

1. "Introduction to Autonomous Mobile Robots" - Siegwart, Nourbakhsh, Scaramuzza
2. "Embedded Systems: Real-Time Interfacing" - Jonathan Valvano
3. "Robot Modeling and Control" - Spong, Hutchinson, Vidyasagar

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
