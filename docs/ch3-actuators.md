---
id: ch3-actuators-physical-ai
title: "Chapter 3: Actuators in Physical AI"
sidebar_label: "3. Actuators in Physical AI"
sidebar_position: 3
description: "Core technical chapter covering actuator types, motor control principles, kinematics basics, motion profiles, and ROS 2 control integration for Physical AI systems."
keywords: [actuators, motors, servo, stepper, bldc, pwm, ros2_control, kinematics, robotics]
---

# Chapter 3: Actuators in Physical AI

> *"Sensors give robots eyes; actuators give them hands and feet."*

---

## 1. Overview / Introduction

In Chapter 1, we established that Physical AI systems operate through continuous **perception-action loops**. In Chapter 2, we explored the perception side through sensors. Now, we complete the loop by examining **actuators**—the devices that enable robots to act upon the world.

Actuators are the muscles of Physical AI. They convert electrical signals, hydraulic pressure, or pneumatic force into physical motion—rotating joints, extending limbs, gripping objects, and generating locomotion. Without actuators, a robot can sense but cannot interact with its environment.

### What You Will Learn

By the end of this chapter, you will be able to:

- **Define actuators** and explain their role in Physical AI systems
- **Categorize actuators** by type: electric, hydraulic, pneumatic, and soft
- **Explain working principles** of key actuators: DC motors, servos, steppers, BLDC motors
- **Understand key properties**: torque, speed, precision, power, efficiency
- **Apply motor control basics**: PWM, position/velocity/torque control
- **Describe kinematics fundamentals**: joint-to-task space mapping
- **Generate motion profiles**: trapezoidal and S-curve trajectories
- **Use ros2_control** for actuator command and feedback
- **Select appropriate actuators** for robotic applications

### Prerequisites

| Concept | Level | Notes |
|---------|-------|-------|
| Chapter 1: Physical AI Introduction | **Required** | Perception-action loop |
| Chapter 2: Sensors | **Required** | Feedback, encoders |
| Basic Physics | Recommended | Force, torque, power (explained) |
| Basic Math | Recommended | Algebra, trigonometry |
| Python Programming | Optional | Helpful for code examples |
| ROS 2 Basics | Recommended | Topics, nodes, messages |

### Chapter Structure

```
1. Overview / Introduction
2. What Are Actuators?
3. Actuator Classification
4. Electric Motor Types
5. Other Actuator Types (Hydraulic, Pneumatic, Soft)
6. Actuator Characteristics & Properties
7. Motor Control Fundamentals
8. Kinematics & Dynamics Introduction
9. Motion Profiles
10. ros2_control Framework
11. Python & ROS 2 Code Examples
12. Real-World Robotics Applications
13. Chapter Summary
14. Glossary
15. Exercises / Review Questions
```

---

## 2. What Are Actuators?

### Definition

An **actuator** is a device that converts control signals (electrical, hydraulic, pneumatic) into physical motion or force.

More formally:

> **Actuator**: A transducer that converts energy from one form (typically electrical, fluid pressure, or compressed air) into mechanical motion (linear or rotational), enabling a robot to interact with and modify its physical environment.

### The Role of Actuators in Physical AI

In the perception-action loop, actuators serve as the **output interface** between the robot's computational decisions and the physical world:

```
┌─────────────────────────────────────────────────────────────────┐
│                    PERCEPTION-ACTION LOOP                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   ┌─────────┐     ┌─────────────┐     ┌───────────┐            │
│   │ SENSORS │────▶│  PERCEPTION │────▶│  DECISION │            │
│   │(Ch. 2)  │     │& PROCESSING │     │ & CONTROL │            │
│   └─────────┘     └─────────────┘     └─────┬─────┘            │
│        ▲                                    │                   │
│        │                                    ▼                   │
│        │                            ┌───────────┐              │
│        │                            │ ACTUATORS │ ◄── Chapter 3│
│        │                            │           │   Focus      │
│        │                            └─────┬─────┘              │
│        │                                  │                     │
│        │        ┌─────────────┐          │                     │
│        └────────│ ENVIRONMENT │◀─────────┘                     │
│                 │   (World)   │                                 │
│                 └─────────────┘                                 │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

Actuators enable robots to:

1. **Move through space**: Locomotion via wheels, legs, propellers, thrusters
2. **Manipulate objects**: Grasping, lifting, placing, assembling
3. **Interact with environments**: Opening doors, pressing buttons, turning handles
4. **Generate forces**: Pushing, pulling, applying controlled pressure
5. **Express behavior**: Gestures, head movements, social interaction

### Sensors + Actuators = Closed-Loop Control

The connection between sensors (Chapter 2) and actuators (Chapter 3) enables **closed-loop control**:

```
                    CLOSED-LOOP CONTROL

    Desired    ┌────────────┐    Control    ┌───────────┐
    State  ───▶│ CONTROLLER │──────────────▶│ ACTUATOR  │
      +        │            │    Signal     │ (Motor)   │
      │        └────────────┘               └─────┬─────┘
      │              ▲                            │
      │              │                            │ Physical
      │         Error│                            │ Motion
      │              │                            ▼
      │        ┌─────┴──────┐              ┌───────────┐
      └────────│  SUBTRACT  │◀─────────────│  SENSOR   │
        -      │            │   Measured   │ (Encoder) │
               └────────────┘    State     └───────────┘
```

Without sensors, actuators operate **open-loop**—executing commands blindly without knowing if the desired motion was achieved. Closed-loop control uses sensor feedback to correct errors continuously.

---

## 3. Actuator Classification

Actuators in Physical AI are classified by their **energy source** and **working principle**:

### Classification Tree

```
                            ACTUATORS
                                │
        ┌───────────────┬───────┴───────┬───────────────┐
        │               │               │               │
    ELECTRIC        HYDRAULIC       PNEUMATIC         SOFT
        │               │               │               │
   ┌────┼────┐      Cylinders       Cylinders      Pneumatic
   │    │    │       Motors         Grippers        Bellows
  DC  Servo Stepper  Valves        Air Muscles     Tendons
   │                                                 SMA
  BLDC
```

### 3.1 Electric Actuators

**Working Principle**: Convert electrical energy into rotational or linear motion via electromagnetic interaction.

**Advantages**:
- Precise control
- High efficiency (75-95%)
- Clean operation
- Wide availability
- Easy integration with digital systems

**Disadvantages**:
- Lower power density than hydraulic
- Heat generation at high loads
- Require gearboxes for high torque

**Applications**: Robot arms, drones, mobile robots, humanoids

### 3.2 Hydraulic Actuators

**Working Principle**: Convert fluid pressure into mechanical force using incompressible liquid (oil).

**Advantages**:
- Very high force/power density
- Smooth motion
- Excellent for heavy lifting
- Self-lubricating

**Disadvantages**:
- Require pumps, reservoirs, valves
- Fluid leaks possible
- Heavy and complex
- Noisy

**Applications**: Industrial robots, excavators, Boston Dynamics Atlas (early versions)

### 3.3 Pneumatic Actuators

**Working Principle**: Convert compressed air pressure into mechanical force.

**Advantages**:
- Fast response
- Clean (uses air)
- Inherently compliant
- Simple on/off control
- Lightweight

**Disadvantages**:
- Lower precision (air is compressible)
- Require air supply
- Limited force control
- Noisy

**Applications**: Grippers, pick-and-place systems, soft robots, industrial automation

### 3.4 Soft Actuators

**Working Principle**: Use compliant materials that deform under pressure, heat, or electrical stimulation.

**Advantages**:
- Inherently safe for HRI
- Conformable grasping
- Lightweight
- Biomimetic motion

**Disadvantages**:
- Lower precision
- Slower response
- Complex modeling
- Limited force

**Applications**: Soft grippers, medical robots, wearable robots, biomimetic systems

### Comparison Summary

| Type | Force Density | Precision | Speed | Efficiency | Complexity |
|------|---------------|-----------|-------|------------|------------|
| Electric | Medium | High | High | High | Low |
| Hydraulic | Very High | Medium | Medium | Medium | High |
| Pneumatic | Low-Medium | Low | Very High | Low | Medium |
| Soft | Low | Low | Low | Low | Medium |

---

## 4. Electric Motor Types

Electric motors dominate robotics due to their precision, efficiency, and controllability. This section covers the major types.

### 4.1 DC Motors (Brushed)

**Working Principle**: Current flows through coils (armature) in a magnetic field, creating torque. Mechanical brushes and commutator switch current direction as the motor rotates.

```
            BRUSHED DC MOTOR

    ┌─────────────────────────────┐
    │    N ─────────────── S      │  Permanent Magnets
    │    │       ┌───┐     │      │
    │    │      ╱     ╲    │      │
    │    │     │       │   │      │  Armature (Rotor)
    │    │      ╲     ╱    │      │  with Windings
    │    │       └───┘     │      │
    │    │         │       │      │
    └────┼─────────┼───────┼──────┘
         │    ┌────┴────┐  │
              │ Brushes │
              │Commutator│
              └─────────┘
                 │   │
                 V+  V-
```

**Characteristics**:
- Simple control (voltage = speed)
- Linear torque-speed relationship
- Brushes wear over time
- EMI from brush sparking

**Control**: Voltage (or PWM) controls speed; polarity controls direction.

**Torque-Speed Relationship**:

`τ = kt × I`
`ω = (V - I × R) / ke`

Where:
- $\tau$ = torque (Nm)
- $k_t$ = torque constant (Nm/A)
- $I$ = current (A)
- $\omega$ = angular velocity (rad/s)
- $V$ = voltage (V)
- $R$ = winding resistance (Ω)
- $k_e$ = back-EMF constant (V/(rad/s))

### 4.2 Brushless DC Motors (BLDC)

**Working Principle**: Permanent magnets on rotor, windings on stator. Electronic commutation replaces mechanical brushes.

```
            BRUSHLESS DC MOTOR

    ┌─────────────────────────────┐
    │   Stator Windings           │
    │    A ────── ●               │
    │            ╱ ╲              │
    │           ╱   ╲             │
    │    C ────●     ●──── B      │
    │          │     │            │
    │          │ N S │            │  Permanent Magnet
    │          │ ┌─┐ │            │  Rotor (inside)
    │          │ │●│ │            │
    │          │ └─┘ │            │
    │          │ S N │            │
    │          └─────┘            │
    └─────────────────────────────┘

    Electronic commutation via Hall sensors or sensorless
```

**Characteristics**:
- Higher efficiency than brushed (no brush losses)
- Longer lifespan (no brush wear)
- Higher power density
- Requires electronic speed controller (ESC)
- More complex control

**Applications**: Drones, electric vehicles, high-performance robots

### 4.3 Servo Motors

**Working Principle**: Motor + gearbox + position sensor (encoder or potentiometer) + feedback controller in one package.

```
            SERVO MOTOR SYSTEM

    ┌─────────────────────────────────────┐
    │                                     │
    │  PWM ──▶ ┌──────────┐              │
    │  Signal  │Controller│              │
    │          └────┬─────┘              │
    │               │                     │
    │               ▼                     │
    │          ┌─────────┐  ┌─────────┐  │
    │          │  Motor  │──│ Gearbox │──┼──▶ Output
    │          └─────────┘  └─────────┘  │    Shaft
    │               ▲                     │
    │               │                     │
    │          ┌─────────┐               │
    │          │Position │               │
    │          │ Sensor  │               │
    │          └─────────┘               │
    │                                     │
    └─────────────────────────────────────┘

    Position command (pulse width) → Position output
```

**Types**:

| Type | Control | Precision | Applications |
|------|---------|-----------|--------------|
| Hobby Servo | PWM pulse width | ~1° | RC, small robots |
| Digital Servo | High-res PWM | ~0.1° | Precision robotics |
| Industrial Servo | Position/velocity/torque | 0.01° | Industrial arms |

**PWM Control for Hobby Servos**:
- Pulse period: 20ms (50 Hz)
- Pulse width: 1ms = 0°, 1.5ms = 90°, 2ms = 180°

```
    PWM Signal for Servo Position

    1ms pulse = 0°          1.5ms pulse = 90°       2ms pulse = 180°

    ┌──┐                    ┌────┐                  ┌──────┐
    │  │                    │    │                  │      │
────┘  └────────────────────┘    └──────────────────┘      └────────

    ◀──────── 20ms ────────▶◀──────── 20ms ────────▶◀───── 20ms ────▶
```

### 4.4 Stepper Motors

**Working Principle**: Rotor moves in discrete angular steps when stator coils are energized in sequence. No feedback required for basic positioning.

```
            STEPPER MOTOR

    ┌─────────────────────────────┐
    │      A+        B+           │  Stator Coils
    │       │         │           │
    │   ────●─────────●────       │
    │       │    ┌─┐  │           │
    │       │    │▲│  │           │  Rotor with
    │   ────●────│ │──●────       │  Teeth/Poles
    │       │    │▼│  │           │
    │       │    └─┘  │           │
    │   ────●─────────●────       │
    │       │         │           │
    │      A-        B-           │
    └─────────────────────────────┘

    Energize coils in sequence: A+ → B+ → A- → B- → A+...
    Each step rotates by step angle (e.g., 1.8° for 200 steps/rev)
```

**Characteristics**:
- Precise open-loop positioning
- Holding torque when stationary
- Discrete steps (typically 1.8° or 0.9°)
- No feedback required for basic use
- Can miss steps under overload

**Step Modes**:

| Mode | Steps/Rev | Torque | Smoothness |
|------|-----------|--------|------------|
| Full Step | 200 | 100% | Rough |
| Half Step | 400 | ~70% | Smoother |
| Microstepping (1/16) | 3200 | ~20% | Very smooth |

**Applications**: 3D printers, CNC machines, positioning systems

### 4.5 Linear Actuators

**Working Principle**: Convert rotational motion to linear motion via leadscrew, ball screw, belt, or rack-and-pinion.

```
            LEADSCREW LINEAR ACTUATOR

    Motor                   Leadscrew            Carriage
    ┌───┐                  ╔═══════════════╗     ┌───┐
    │   │──────────────────║               ║─────│   │──▶ Linear
    │ M │     Coupling     ║    Threads    ║ Nut │   │    Motion
    │   │──────────────────║               ║─────│   │
    └───┘                  ╚═══════════════╝     └───┘

    Motor rotation → Screw rotation → Nut linear motion
```

**Types**:

| Mechanism | Efficiency | Speed | Precision | Backdrivability |
|-----------|------------|-------|-----------|-----------------|
| Leadscrew | 30-50% | Low | High | No (self-locking) |
| Ball Screw | 90%+ | Medium | Very High | Yes |
| Belt Drive | 95%+ | High | Medium | Yes |
| Rack & Pinion | 95%+ | High | Medium | Yes |

---

## 5. Other Actuator Types

### 5.1 Hydraulic Actuators

**Components**:
- Pump (generates pressure)
- Reservoir (stores fluid)
- Valves (control flow)
- Cylinder or motor (converts pressure to motion)

```
            HYDRAULIC CYLINDER

         Fluid In                    Fluid Out
            ▼                           ▲
    ┌───────┴───────────────────────────┴───────┐
    │       │███████████████████████████│       │
    │       │███████ PISTON ████████████│       │
    │       │███████████████████████████│───────┼──▶ Rod
    │       │                           │       │    (Output)
    │       │                           │       │
    └───────────────────────────────────────────┘

    Pressure × Area = Force
    F = P × A
```

**Force Calculation**:

`F = P × A`

Where:
- $F$ = force (N)
- $P$ = pressure (Pa or N/m²)
- $A$ = piston area (m²)

**Example**: 20 MPa pressure × 0.01 m² area = 200,000 N (20 tons!)

### 5.2 Pneumatic Actuators

**Components**:
- Compressor (generates compressed air)
- Tank (stores air)
- Valves (control flow)
- Cylinder or gripper (converts pressure to motion)

**Characteristics**:
- Faster than hydraulic (air is lighter)
- Less precise (air is compressible)
- Clean (no oil leaks)
- Inherently compliant (air acts as spring)

**Common Application: Pneumatic Gripper**

```
            PNEUMATIC GRIPPER

              Air Supply
                  │
                  ▼
    ┌─────────────┴─────────────┐
    │         Cylinder          │
    │  ┌───────────────────┐    │
    │  │    Piston ←─ Air  │    │
    │  └─────────┬─────────┘    │
    │            │              │
    │    ┌───────┴───────┐      │
    │    │    Linkage    │      │
    │    └───────┬───────┘      │
    │      ╱           ╲        │
    │     ╱             ╲       │
    └────╱───────────────╲──────┘
        ▼                 ▼
      Jaw 1             Jaw 2

    Air pressure → Piston motion → Jaws close
```

### 5.3 Soft Actuators

**Types and Mechanisms**:

| Type | Mechanism | Response Time | Force |
|------|-----------|---------------|-------|
| Pneumatic Elastomer | Air chambers expand | Fast | Medium |
| McKibben Muscle | Air contracts braided tube | Fast | High |
| Tendon-Driven | Cable pulls | Fast | Medium |
| Shape Memory Alloy | Heat contracts wire | Slow | Low |
| Electroactive Polymer | Electric field deforms | Medium | Low |

**Pneumatic Soft Actuator Example**:

```
            PNEUNETS BENDING ACTUATOR

    Top Layer (Inextensible)
    ════════════════════════════
    ┌──┐ ┌──┐ ┌──┐ ┌──┐ ┌──┐     Air Chambers
    │  │ │  │ │  │ │  │ │  │     (expand with pressure)
    └──┘ └──┘ └──┘ └──┘ └──┘
    ════════════════════════════
    Bottom Layer (Inextensible)

                    ↓ Pressurize

            ╭────────────────╮
           ╱  ╱  ╱  ╱  ╱  ╱  ╲    Chambers expand,
          ╱──╱──╱──╱──╱──╱────╲   bottom constrained,
         ╱                    ╲   actuator BENDS
        ╱                      ╲
```

---

## 6. Actuator Characteristics & Properties

### 6.1 Torque

**Definition**: Rotational force; the ability to cause angular acceleration.

`τ = F × r`

Where:
- $\tau$ = torque (Nm)
- $F$ = force (N)
- $r$ = moment arm / radius (m)

**Motor Torque Specifications**:

| Specification | Description |
|---------------|-------------|
| Stall Torque | Maximum torque at zero speed |
| Continuous Torque | Torque sustainable indefinitely |
| Peak Torque | Brief overload torque (seconds) |
| Torque Constant ($k_t$) | Torque per amp (Nm/A) |

### 6.2 Speed

**Definition**: Angular velocity; how fast the actuator rotates.

**Units**: RPM (revolutions per minute) or rad/s

**Conversion**:
The conversion formula is: `ω (rad/s) = (2π × RPM) / 60`

**Motor Speed Specifications**:

| Specification | Description |
|---------------|-------------|
| No-Load Speed | Maximum speed with no load |
| Rated Speed | Speed at continuous torque |
| Speed Constant ($k_v$) | RPM per volt (for BLDC) |

### 6.3 Torque-Speed Curve

The fundamental relationship between torque and speed for DC motors:

```
            TORQUE-SPEED CURVE

    Torque (Nm)
        │
    τ_s ┼─────────●                    Stall Torque
        │          ╲                   (max torque, zero speed)
        │           ╲
        │            ╲
        │             ╲
        │              ╲
        │               ╲
        │                ╲
        │                 ╲
        │                  ╲
    0   ┼───────────────────●────── Speed (RPM)
        0                   ω_nl
                            No-Load Speed
                            (max speed, zero torque)

    Linear relationship: τ = τ_s × (1 - ω/ω_nl)

    Operating Point: Where motor load curve intersects
```

**Power Output**:

`P = τ × ω`

Maximum power occurs at 50% of stall torque and 50% of no-load speed.

### 6.4 Gear Ratios

Gearboxes trade speed for torque:

`τ_out = τ_in × N × η`

`ω_out = ω_in / N`

Where:
- $N$ = gear ratio
- $\eta$ = efficiency (typically 0.85-0.95)

**Example**: 100:1 gearbox
- Output torque: 100× input torque (minus losses)
- Output speed: 1/100 of input speed

**Tradeoff Table**:

| Gear Ratio | Torque | Speed | Precision | Backdrivability |
|------------|--------|-------|-----------|-----------------|
| Low (5:1) | Low | High | Lower | Yes |
| Medium (50:1) | Medium | Medium | Medium | Marginal |
| High (100:1+) | High | Low | Higher | No |

### 6.5 Efficiency

**Definition**: Ratio of mechanical output power to electrical input power.

`η = P_out / P_in = (τ × ω) / (V × I)`

**Typical Efficiencies**:

| Motor Type | Efficiency |
|------------|------------|
| Brushed DC | 70-85% |
| Brushless DC | 85-95% |
| Stepper | 60-80% |
| Servo | 80-90% |
| Hydraulic | 70-85% |
| Pneumatic | 20-30% |

### 6.6 Bandwidth

**Definition**: Frequency response; how fast the actuator can respond to changing commands.

**Factors Limiting Bandwidth**:
- Mechanical inertia
- Friction
- Controller sampling rate
- Communication delays

**Typical Bandwidths**:

| Application | Required Bandwidth |
|-------------|-------------------|
| Position control | 10-50 Hz |
| Velocity control | 50-200 Hz |
| Force control | 100-1000 Hz |
| Humanoid balance | 500-2000 Hz |

---

## 7. Motor Control Fundamentals

### 7.1 PWM (Pulse Width Modulation)

**Principle**: Vary average voltage by rapidly switching between full ON and OFF.

```
            PWM SIGNALS

    25% Duty Cycle (Low Speed)
    ┌─┐   ┌─┐   ┌─┐   ┌─┐
    │ │   │ │   │ │   │ │
────┘ └───┘ └───┘ └───┘ └───
    V_avg = 0.25 × V_max

    50% Duty Cycle (Medium Speed)
    ┌──┐  ┌──┐  ┌──┐  ┌──┐
    │  │  │  │  │  │  │  │
────┘  └──┘  └──┘  └──┘  └──
    V_avg = 0.50 × V_max

    75% Duty Cycle (High Speed)
    ┌───┐ ┌───┐ ┌───┐ ┌───┐
    │   │ │   │ │   │ │   │
────┘   └─┘   └─┘   └─┘   └─
    V_avg = 0.75 × V_max
```

**Advantages of PWM**:
- High efficiency (transistors fully ON or OFF)
- Precise digital control
- Low heat dissipation
- Wide speed range

**PWM Frequency**:
- Too low: Motor vibrates, audible noise
- Too high: Switching losses, EMI
- Typical: 1-20 kHz for motors

### 7.2 H-Bridge Motor Driver

**Purpose**: Control motor direction and speed using PWM.

```
            H-BRIDGE CIRCUIT

         V+
          │
    ┌─────┼─────┐
    │     │     │
   ┌┴┐   │    ┌┴┐
   │Q1│   │    │Q2│   High-Side
   │  │   │    │  │   Switches
   └┬┘   │    └┬┘
    │     │     │
    ├─────┼─────┤
    │           │
    │   ┌───┐   │
    │───│ M │───│     Motor
    │   └───┘   │
    │           │
    ├─────┼─────┤
    │     │     │
   ┌┴┐   │    ┌┴┐
   │Q3│   │    │Q4│   Low-Side
   │  │   │    │  │   Switches
   └┬┘   │    └┬┘
    │     │     │
    └─────┼─────┘
          │
         GND

    Forward: Q1+Q4 ON, Q2+Q3 OFF
    Reverse: Q2+Q3 ON, Q1+Q4 OFF
    Brake:   Q1+Q2 ON or Q3+Q4 ON
    Coast:   All OFF
```

### 7.3 Position Control

**Feedback Loop**: Use encoder feedback to achieve precise positioning.

```
            POSITION CONTROL LOOP

    θ_desired   ┌────────────┐   Torque    ┌───────┐   θ_actual
    ────────────▶│    PID     │────────────▶│ Motor │────────────▶
         +      │ Controller │   Command   │       │
         │      └────────────┘             └───┬───┘
         │            ▲                        │
         │            │                        │
         │       ┌────┴─────┐                 │
         └───────│ Subtract │◀────────────────┘
           -     │          │  θ_measured
                 └──────────┘
                       ▲
                       │
                 ┌─────┴─────┐
                 │  Encoder  │
                 └───────────┘
```

**PID Controller**:

`u(t) = Kp × e(t) + Ki × ∫e(t)dt + Kd × de(t)/dt`

Where:
- $e(t)$ = position error (desired - actual)
- $K_p$ = proportional gain (reacts to current error)
- $K_i$ = integral gain (eliminates steady-state error)
- $K_d$ = derivative gain (damps oscillations)

### 7.4 Velocity Control

**Purpose**: Control motor speed rather than position.

**Velocity Feedback Loop**:
- Measure velocity from encoder (differentiate position or use tachometer)
- Compare to desired velocity
- Apply PID control

**Feedforward + Feedback**:

`u = K_ff × ω_desired + Kp × e_ω + Ki × ∫e_ω dt`

Feedforward term (K_ff) provides approximate command; feedback corrects errors.

### 7.5 Torque/Force Control

**Purpose**: Control motor torque (force) rather than position or velocity.

**Implementation**: Control motor current (torque ∝ current for most motors).

`τ = kt × I`

**Current Control Loop**:

```
    τ_desired    ┌───────────┐    PWM     ┌───────┐   τ_actual
    ─────────────▶│  Current  │───────────▶│ Motor │────────────▶
         +       │Controller │           │       │
         │       └───────────┘           └───┬───┘
         │             ▲                     │
         │             │                     │
         │        ┌────┴────┐               │
         └────────│Subtract │◀──────────────┘
           -      │         │  I_measured
                  └─────────┘
                       ▲
                       │
                  ┌────┴────┐
                  │ Current │
                  │ Sensor  │
                  └─────────┘
```

**Applications**: Force-controlled manipulation, compliant motion, haptic feedback

---

## 8. Kinematics & Dynamics Introduction

### 8.1 Joint Space vs. Task Space

**Joint Space**: Describes robot state in terms of joint angles/positions.
- Example: θ₁ = 30°, θ₂ = 45°, θ₃ = -20°

**Task Space** (Cartesian Space): Describes end-effector position and orientation.
- Example: x = 0.5m, y = 0.3m, z = 0.2m, roll = 0°, pitch = 10°, yaw = 45°

```
            JOINT SPACE vs TASK SPACE

    JOINT SPACE                      TASK SPACE
    (Actuator View)                  (User View)

         θ₁                              z
          │                              │
          │╲                             │    ● End Effector
          │ ╲ Link 1                     │   ╱  (x, y, z)
          │  ╲                           │  ╱
          ●───╲                          │ ╱
         Base  ╲ θ₂                      │╱
                ╲│                       ●────────── y
                 ╲                      ╱
                  ╲ Link 2             ╱
                   ╲                  ╱
                    ●                x
               End Effector

    Forward Kinematics: Joint angles → End effector position
    Inverse Kinematics: End effector position → Joint angles
```

### 8.2 Forward Kinematics

**Definition**: Compute end-effector position/orientation from joint angles.

**Simple 2-Link Planar Arm Example**:

```
                  Link 2 (L₂)
                 ╱
                ╱ θ₂
               ●──────────● End Effector
              ╱           (x, y)
             ╱
    Link 1 (L₁)
           ╱
          ╱ θ₁
         ●
        Base
        (0, 0)
```

**Forward Kinematics Equations**:

`x = L₁ × cos(θ₁) + L₂ × cos(θ₁ + θ₂)`

`y = L₁ × sin(θ₁) + L₂ × sin(θ₁ + θ₂)`

### 8.3 Inverse Kinematics

**Definition**: Compute joint angles to achieve desired end-effector position.

**Challenges**:
- Multiple solutions (elbow up vs. elbow down)
- No solution (position outside workspace)
- Singularities (infinite solutions)

**2-Link Arm Inverse Kinematics**:

`cos(θ₂) = (x² + y² - L₁² - L₂²) / (2 × L₁ × L₂)`

`θ₂ = ±arccos((x² + y² - L₁² - L₂²) / (2 × L₁ × L₂))`

`θ₁ = arctan2(y, x) - arctan2(L₂ × sin(θ₂), L₁ + L₂ × cos(θ₂))`

The ± gives two solutions (elbow up/down).

### 8.4 Degrees of Freedom (DOF)

**Definition**: Number of independent parameters needed to specify robot configuration.

**Common Robot DOF**:

| Robot Type | Typical DOF | Description |
|------------|-------------|-------------|
| Mobile base | 2-3 | (x, y) or (x, y, θ) |
| SCARA arm | 4 | 2R + 1P + 1R |
| 6-axis arm | 6 | Full 3D pose control |
| Humanoid arm | 7+ | Redundant for obstacles |
| Humanoid (full) | 30+ | Legs, arms, torso, head |

### 8.5 Dynamics Basics

**Dynamics**: Relationship between forces/torques and motion.

**Newton-Euler Equation for Rotational Motion**:

`τ = J × α + b × ω + τ_gravity + τ_friction`

Where:
- $\tau$ = applied torque
- $J$ = moment of inertia
- $\alpha$ = angular acceleration
- $b$ = viscous damping
- $\omega$ = angular velocity
- τ_gravity = gravity torque
- τ_friction = friction torque

**Key Insight**: To accelerate a joint, you must provide torque to overcome:
1. Inertia (mass × acceleration)
2. Gravity (supporting the link weight)
3. Friction (Coulomb and viscous)
4. External loads

---

## 9. Motion Profiles

### 9.1 Why Motion Profiles?

Moving directly from point A to point B with step commands causes:
- Infinite acceleration (jerky motion)
- Mechanical stress
- Vibration and overshoot
- Poor tracking

**Solution**: Plan smooth motion profiles that respect physical limits.

### 9.2 Trapezoidal Velocity Profile

The most common motion profile: accelerate → cruise → decelerate.

```
            TRAPEZOIDAL VELOCITY PROFILE

    Velocity
        │
    v_max├─────────────────────────┐
        │         ╱               │╲
        │        ╱                │ ╲
        │       ╱                 │  ╲
        │      ╱                  │   ╲
        │     ╱                   │    ╲
        │    ╱                    │     ╲
    0   ┼───╱─────────────────────┼──────╲───── Time
        0  t₁                    t₂      t₃

        │◀─▶│◀────────────────────▶│◀────▶│
        Accel    Constant Velocity   Decel
        Phase         Phase          Phase
```

**Equations**:

**Acceleration phase** (0 ≤ t ≤ t₁):
- `v(t) = a_max × t`
- `s(t) = (1/2) × a_max × t²`

**Cruise phase** (t₁ ≤ t ≤ t₂):
- `v(t) = v_max`
- `s(t) = s₁ + v_max × (t - t₁)`

**Deceleration phase** (t₂ ≤ t ≤ t₃):
- `v(t) = v_max - a_max × (t - t₂)`
- `s(t) = s₂ + v_max × (t - t₂) - (1/2) × a_max × (t - t₂)²`

### 9.3 S-Curve (Jerk-Limited) Profile

Smoother than trapezoidal: limits jerk (rate of acceleration change).

```
            S-CURVE VELOCITY PROFILE

    Velocity
        │
    v_max├───────────────────────────────╮
        │                    ╭───────────╯
        │              ╭─────╯
        │         ╭────╯
        │    ╭────╯
        │╭───╯
    0   ┼╯─────────────────────────────────── Time
        0

    Smooth transitions (no sharp corners)
    Continuous acceleration (no steps)
    Finite jerk (no infinite acceleration change)
```

**Advantages**:
- Reduced vibration
- Lower mechanical stress
- Better trajectory tracking
- Smoother motion appearance

**Disadvantage**: Longer motion time than trapezoidal for same distance.

### 9.4 Motion Profile Parameters

| Parameter | Symbol | Unit | Description |
|-----------|--------|------|-------------|
| Max Velocity | v_max | m/s or rad/s | Speed limit |
| Max Acceleration | a_max | m/s² or rad/s² | Acceleration limit |
| Max Jerk | j_max | m/s³ or rad/s³ | Jerk limit (S-curve) |
| Travel Distance | d | m or rad | Total motion |
| Motion Time | T | s | Total duration |

---

## 10. ros2_control Framework

### 10.1 Overview

**ros2_control** is the standard ROS 2 framework for real-time robot control. It provides:
- Hardware abstraction layer
- Controller interface standardization
- Real-time control loop management

```
            ros2_control ARCHITECTURE

    ┌─────────────────────────────────────────────────────────┐
    │                    USER APPLICATIONS                    │
    │    (Navigation, Manipulation, Motion Planning)          │
    └───────────────────────┬─────────────────────────────────┘
                            │ Commands (Position, Velocity)
                            ▼
    ┌─────────────────────────────────────────────────────────┐
    │                  CONTROLLER MANAGER                     │
    │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
    │  │  Position    │  │  Velocity    │  │  Trajectory  │  │
    │  │  Controller  │  │  Controller  │  │  Controller  │  │
    │  └──────────────┘  └──────────────┘  └──────────────┘  │
    └───────────────────────┬─────────────────────────────────┘
                            │ Joint Commands/States
                            ▼
    ┌─────────────────────────────────────────────────────────┐
    │                  HARDWARE INTERFACE                     │
    │           (Abstraction Layer for Actuators)             │
    │  ┌─────────────────────┐  ┌─────────────────────┐      │
    │  │   Command Interface │  │   State Interface   │      │
    │  │  (position/velocity │  │  (position/velocity │      │
    │  │   /effort)          │  │   /effort)          │      │
    │  └─────────────────────┘  └─────────────────────┘      │
    └───────────────────────┬─────────────────────────────────┘
                            │
            ┌───────────────┼───────────────┐
            ▼               ▼               ▼
    ┌───────────┐   ┌───────────┐   ┌───────────┐
    │  Gazebo   │   │   Isaac   │   │   Real    │
    │    Sim    │   │    Sim    │   │  Hardware │
    └───────────┘   └───────────┘   └───────────┘
```

### 10.2 Hardware Interface Types

| Interface | Description | Use Case |
|-----------|-------------|----------|
| `position` | Desired joint position | Positioning tasks |
| `velocity` | Desired joint velocity | Continuous motion |
| `effort` | Desired torque/force | Force control |

### 10.3 URDF Configuration for ros2_control

```xml
<!-- Example: Robot arm URDF with ros2_control -->
<robot name="robot_arm">

  <!-- Link and Joint definitions... -->

  <ros2_control name="robot_arm_control" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>

    <joint name="joint_1">
      <command_interface name="position">
        <param name="min">-3.14</param>
        <param name="max">3.14</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

    <joint name="joint_2">
      <command_interface name="position">
        <param name="min">-1.57</param>
        <param name="max">1.57</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

    <!-- More joints... -->

  </ros2_control>

</robot>
```

### 10.4 Controller Configuration (YAML)

```yaml
# controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
```

---

## 11. Python & ROS 2 Code Examples

### 11.1 Simple Motor Control Simulation (Pure Python)

```python
"""
DC Motor Simulation with PWM Control
Demonstrates basic motor dynamics and PWM speed control
"""

import math

class DCMotor:
    """Simple DC motor model."""

    def __init__(self, R=1.0, L=0.01, kt=0.1, ke=0.1, J=0.01, b=0.001):
        """
        Initialize motor parameters.

        Args:
            R: Armature resistance (ohms)
            L: Armature inductance (H)
            kt: Torque constant (Nm/A)
            ke: Back-EMF constant (V/(rad/s))
            J: Rotor inertia (kg.m^2)
            b: Viscous friction (Nm/(rad/s))
        """
        self.R = R
        self.L = L
        self.kt = kt
        self.ke = ke
        self.J = J
        self.b = b

        # State variables
        self.current = 0.0      # Armature current (A)
        self.velocity = 0.0     # Angular velocity (rad/s)
        self.position = 0.0     # Angular position (rad)

    def step(self, voltage, dt, load_torque=0.0):
        """
        Simulate one time step.

        Args:
            voltage: Applied voltage (V)
            dt: Time step (s)
            load_torque: External load torque (Nm)

        Returns:
            Dictionary with current state
        """
        # Electrical dynamics: V = I*R + L*dI/dt + ke*omega
        back_emf = self.ke * self.velocity
        di_dt = (voltage - self.current * self.R - back_emf) / self.L
        self.current += di_dt * dt

        # Mechanical dynamics: J*d_omega/dt = kt*I - b*omega - T_load
        motor_torque = self.kt * self.current
        d_omega_dt = (motor_torque - self.b * self.velocity - load_torque) / self.J
        self.velocity += d_omega_dt * dt

        # Position integration
        self.position += self.velocity * dt

        return {
            'current': self.current,
            'velocity': self.velocity,
            'position': self.position,
            'torque': motor_torque
        }


def pwm_voltage(duty_cycle, v_supply=12.0):
    """Convert PWM duty cycle (0-1) to average voltage."""
    return duty_cycle * v_supply


# Simulation
motor = DCMotor()
dt = 0.001  # 1ms time step
t_total = 2.0  # 2 seconds

print("=== DC Motor PWM Control Simulation ===")
print(f"{'Time':>6} | {'Duty%':>6} | {'Velocity':>10} | {'Current':>8} | {'RPM':>8}")
print("-" * 55)

t = 0
while t < t_total:
    # Change duty cycle at 1 second
    if t < 1.0:
        duty = 0.5  # 50% duty cycle
    else:
        duty = 0.8  # 80% duty cycle

    voltage = pwm_voltage(duty)
    state = motor.step(voltage, dt)

    # Print every 100ms
    if abs(t % 0.1) < dt:
        rpm = state['velocity'] * 60 / (2 * math.pi)
        print(f"{t:6.2f} | {duty*100:5.0f}% | {state['velocity']:10.2f} | "
              f"{state['current']:8.3f} | {rpm:8.1f}")

    t += dt

print("-" * 55)
print("Note: Velocity increases with higher duty cycle (more voltage)")
```

### 11.2 Trapezoidal Motion Profile Generator

```python
"""
Trapezoidal Motion Profile Generator
Generates position, velocity, and acceleration profiles
"""

def generate_trapezoidal_profile(distance, v_max, a_max, dt=0.01):
    """
    Generate a trapezoidal velocity motion profile.

    Args:
        distance: Total distance to travel (units)
        v_max: Maximum velocity (units/s)
        a_max: Maximum acceleration (units/s²)
        dt: Time step (s)

    Returns:
        Lists of (time, position, velocity, acceleration)
    """
    # Calculate profile parameters
    t_accel = v_max / a_max  # Time to accelerate to v_max
    d_accel = 0.5 * a_max * t_accel**2  # Distance during acceleration

    # Check if we can reach v_max (triangular vs trapezoidal)
    if 2 * d_accel > distance:
        # Triangular profile (can't reach v_max)
        t_accel = math.sqrt(distance / a_max)
        t_cruise = 0
        v_peak = a_max * t_accel
    else:
        # Trapezoidal profile
        d_cruise = distance - 2 * d_accel
        t_cruise = d_cruise / v_max
        v_peak = v_max

    t_total = 2 * t_accel + t_cruise

    # Generate profile
    times = []
    positions = []
    velocities = []
    accelerations = []

    t = 0
    while t <= t_total + dt:
        if t <= t_accel:
            # Acceleration phase
            a = a_max
            v = a_max * t
            s = 0.5 * a_max * t**2
        elif t <= t_accel + t_cruise:
            # Cruise phase
            a = 0
            v = v_peak
            s = d_accel + v_peak * (t - t_accel)
        else:
            # Deceleration phase
            t_decel = t - t_accel - t_cruise
            a = -a_max
            v = v_peak - a_max * t_decel
            s = d_accel + v_peak * t_cruise + v_peak * t_decel - 0.5 * a_max * t_decel**2

        times.append(t)
        positions.append(s)
        velocities.append(max(0, v))  # Clamp to non-negative
        accelerations.append(a)

        t += dt

    return times, positions, velocities, accelerations


# Example usage
import math

distance = 10.0  # units (e.g., radians or meters)
v_max = 5.0      # units/s
a_max = 10.0     # units/s²

times, positions, velocities, accelerations = generate_trapezoidal_profile(
    distance, v_max, a_max
)

print("=== Trapezoidal Motion Profile ===")
print(f"Distance: {distance}, v_max: {v_max}, a_max: {a_max}")
print(f"Total time: {times[-1]:.2f}s")
print()
print(f"{'Time':>6} | {'Position':>10} | {'Velocity':>10} | {'Accel':>10}")
print("-" * 50)

# Print every 0.1 seconds
for i, t in enumerate(times):
    if abs(t % 0.2) < 0.015:
        print(f"{t:6.2f} | {positions[i]:10.3f} | {velocities[i]:10.3f} | "
              f"{accelerations[i]:10.3f}")
```

### 11.3 ROS 2 Joint State Subscriber

```python
#!/usr/bin/env python3
"""
ROS 2 Joint State Subscriber
Subscribes to /joint_states and prints joint positions
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateSubscriber(Node):
    """Node that subscribes to joint states."""

    def __init__(self):
        super().__init__('joint_state_subscriber')

        # Create subscription
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10  # QoS depth
        )

        self.get_logger().info('Joint State Subscriber started')

    def joint_state_callback(self, msg):
        """Process incoming joint state messages."""
        self.get_logger().info('--- Joint States ---')

        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else 0.0
            velocity = msg.velocity[i] if i < len(msg.velocity) else 0.0
            effort = msg.effort[i] if i < len(msg.effort) else 0.0

            self.get_logger().info(
                f'{name}: pos={position:.3f} rad, '
                f'vel={velocity:.3f} rad/s, '
                f'effort={effort:.3f} Nm'
            )


def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 11.4 ROS 2 Joint Trajectory Publisher

```python
#!/usr/bin/env python3
"""
ROS 2 Joint Trajectory Publisher
Sends a trajectory to move a robot arm
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class TrajectoryPublisher(Node):
    """Node that sends joint trajectory commands."""

    def __init__(self):
        super().__init__('trajectory_publisher')

        # Create action client
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server available!')

    def send_trajectory(self, joint_names, waypoints):
        """
        Send a trajectory to the robot.

        Args:
            joint_names: List of joint names
            waypoints: List of (positions, time_from_start) tuples
        """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names

        for positions, time_sec in waypoints:
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0] * len(positions)  # Zero velocity at waypoints
            point.time_from_start = Duration(sec=int(time_sec),
                                             nanosec=int((time_sec % 1) * 1e9))
            goal_msg.trajectory.points.append(point)

        self.get_logger().info('Sending trajectory...')
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted, executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handle trajectory result."""
        result = future.result().result
        self.get_logger().info(f'Trajectory complete! Error code: {result.error_code}')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()

    # Define trajectory
    joint_names = ['joint_1', 'joint_2', 'joint_3']

    # Waypoints: (positions in radians, time in seconds)
    waypoints = [
        ([0.0, 0.0, 0.0], 0.0),      # Start position
        ([0.5, 0.3, -0.2], 2.0),     # First waypoint at 2s
        ([1.0, 0.6, -0.4], 4.0),     # Second waypoint at 4s
        ([0.0, 0.0, 0.0], 6.0),      # Return to start at 6s
    ]

    node.send_trajectory(joint_names, waypoints)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 11.5 Simple Inverse Kinematics (2-Link Arm)

```python
"""
Simple 2-Link Planar Arm Inverse Kinematics
Computes joint angles to reach a target position
"""

import math

def forward_kinematics(theta1, theta2, L1, L2):
    """
    Compute end-effector position from joint angles.

    Args:
        theta1, theta2: Joint angles (radians)
        L1, L2: Link lengths

    Returns:
        (x, y) end-effector position
    """
    x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
    return x, y


def inverse_kinematics(x, y, L1, L2, elbow_up=True):
    """
    Compute joint angles to reach target position.

    Args:
        x, y: Target position
        L1, L2: Link lengths
        elbow_up: True for elbow-up solution, False for elbow-down

    Returns:
        (theta1, theta2) in radians, or None if unreachable
    """
    # Check if target is reachable
    d = math.sqrt(x**2 + y**2)
    if d > L1 + L2 or d < abs(L1 - L2):
        return None  # Target out of reach

    # Compute theta2 using law of cosines
    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = max(-1, min(1, cos_theta2))  # Clamp for numerical stability

    if elbow_up:
        theta2 = math.acos(cos_theta2)
    else:
        theta2 = -math.acos(cos_theta2)

    # Compute theta1
    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)

    return theta1, theta2


# Example usage
L1, L2 = 1.0, 0.8  # Link lengths

print("=== 2-Link Arm Inverse Kinematics ===")
print(f"Link lengths: L1={L1}, L2={L2}")
print(f"Workspace: {abs(L1-L2):.1f} to {L1+L2:.1f}")
print()

# Test targets
targets = [(1.2, 0.5), (0.5, 1.0), (1.5, 0.0), (0.3, 0.3)]

for x, y in targets:
    print(f"Target: ({x:.1f}, {y:.1f})")

    result = inverse_kinematics(x, y, L1, L2, elbow_up=True)

    if result is None:
        print("  -> UNREACHABLE")
    else:
        theta1, theta2 = result
        # Verify with forward kinematics
        x_fk, y_fk = forward_kinematics(theta1, theta2, L1, L2)

        print(f"  -> theta1={math.degrees(theta1):.1f}°, theta2={math.degrees(theta2):.1f}°")
        print(f"  -> Verification: ({x_fk:.3f}, {y_fk:.3f})")
        print(f"  -> Error: {math.sqrt((x-x_fk)**2 + (y-y_fk)**2):.6f}")
    print()
```

---

## 12. Real-World Robotics Applications

### 12.1 Robot Arm Actuators

**Typical 6-DOF Industrial Arm Configuration**:

```
            6-DOF ROBOT ARM ACTUATORS

                    ● End Effector (Gripper)
                   ╱
                  ╱  Joint 6: Wrist Roll
                 ●───────────────────
                 │   Joint 5: Wrist Pitch
                 │
            ─────●──── Joint 4: Wrist Yaw
                │
                │
                │    Joint 3: Elbow
            ────●────
               ╱
              ╱      Joint 2: Shoulder Pitch
             ●───────
             │
             │       Joint 1: Base Rotation
        ─────●─────
             │
        ▓▓▓▓▓▓▓▓▓▓▓  Base
```

**Actuator Selection by Joint**:

| Joint | Motion | Torque Need | Typical Actuator |
|-------|--------|-------------|------------------|
| Base (J1) | Rotation | High | Servo + 100:1 gearbox |
| Shoulder (J2) | Pitch | Very High | Servo + 150:1 gearbox |
| Elbow (J3) | Pitch | High | Servo + 100:1 gearbox |
| Wrist (J4-J6) | Roll/Pitch/Yaw | Low | Servo + 50:1 gearbox |
| Gripper | Open/Close | Low | Small servo or pneumatic |

### 12.2 Humanoid Robot Leg

**Walking Requires Complex Actuation**:

```
            HUMANOID LEG JOINTS

            Hip ───●─── 3 DOF (Roll, Pitch, Yaw)
                   │
                   │
                   │
           Knee ───●─── 1 DOF (Pitch)
                   │
                   │
                   │
          Ankle ───●─── 2 DOF (Pitch, Roll)
                   │
                  ▓▓▓  Foot

    Total: 6 DOF per leg = 12 DOF for bipedal locomotion
```

**Challenges**:
- High torque at hip and knee (supporting body weight)
- Fast response for balance (>500 Hz control)
- Energy efficiency (limited battery)
- Backdrivability for compliance

**Solutions**:
- Series Elastic Actuators (SEA): Spring between motor and joint
- Quasi-Direct Drive: Low gear ratio (4:1-10:1) with high-torque motor
- Hydraulic (Atlas early): Maximum power density

### 12.3 Quadcopter Drone Propulsion

**Thrust Generation**:

```
            QUADCOPTER MOTOR LAYOUT

                  Front
                    │
            M1 ─────┼───── M2
          (CCW)     │     (CW)
                    │
             ───────●───────  Center
                    │
            M4 ─────┼───── M3
          (CW)      │     (CCW)
                    │
                  Rear

    M1, M3: Counter-clockwise rotation
    M2, M4: Clockwise rotation

    Thrust control:
    - Altitude: All motors increase/decrease equally
    - Pitch: Front vs rear differential
    - Roll: Left vs right differential
    - Yaw: CW vs CCW motor differential
```

**Motor Requirements**:

| Specification | Typical Value |
|---------------|---------------|
| Type | Brushless DC (BLDC) |
| KV Rating | 800-2500 RPM/V |
| Thrust/Weight | 2:1 minimum |
| Response | &lt;50ms to full throttle |
| ESC PWM | 400-500 Hz |

### 12.4 Robotic Gripper Types

**Comparison of Gripper Actuators**:

| Type | Actuator | Pros | Cons |
|------|----------|------|------|
| 2-Finger Parallel | Servo/Pneumatic | Simple, strong | Limited shapes |
| 3-Finger Adaptive | Servo | Versatile grasping | Complex control |
| Vacuum | Pneumatic | Fast, flat objects | Surface dependent |
| Soft Gripper | Pneumatic | Safe, adaptive | Lower force |
| Dexterous Hand | Many servos | Human-like | Very complex |

**Parallel Gripper Example**:

```
            PARALLEL JAW GRIPPER

         Pneumatic
         Cylinder
            │
            ▼
    ┌───────┬───────┐
    │       │       │
    │   ◄───┼───►   │  Piston drives
    │       │       │  jaws symmetrically
    ├───────┴───────┤
    │               │
    │  ▼         ▼  │
    └───┘       └───┘
     Jaw 1      Jaw 2

    Stroke: 20-100mm typical
    Force: 50-500N typical
    Speed: Open/close in 0.1-0.5s
```

---

## 13. Chapter Summary

### Key Takeaways

1. **Actuators Complete the Loop**: Sensors enable perception; actuators enable action. Together they form the perception-action loop.

2. **Four Main Actuator Types**:
   - **Electric**: Precise, efficient, most common
   - **Hydraulic**: High force, heavy industrial
   - **Pneumatic**: Fast, compliant, binary control
   - **Soft**: Safe, adaptive, emerging technology

3. **Electric Motor Types**:
   - **Brushed DC**: Simple, inexpensive
   - **Brushless DC**: Efficient, long-life
   - **Servo**: Position feedback integrated
   - **Stepper**: Precise open-loop positioning

4. **Key Properties**:
   - **Torque**: Rotational force capability
   - **Speed**: Angular velocity
   - **Power**: Torque × Speed
   - **Efficiency**: Output/Input power
   - **Bandwidth**: Response frequency

5. **Motor Control Fundamentals**:
   - **PWM**: Digital speed control
   - **Position Control**: Encoder feedback + PID
   - **Velocity Control**: Speed regulation
   - **Torque Control**: Current control

6. **Kinematics**:
   - **Forward**: Joint angles → End-effector position
   - **Inverse**: Desired position → Joint angles
   - **DOF**: Determines robot capability

7. **Motion Profiles**:
   - **Trapezoidal**: Accel → Cruise → Decel
   - **S-Curve**: Jerk-limited, smoother

8. **ros2_control**:
   - Hardware abstraction layer
   - Controller manager
   - Same code for simulation and real hardware

### What's Next: Chapter 4 - Control Systems

In Chapter 4, we dive deep into **control systems**—the algorithms that connect sensors and actuators:

- Feedback control theory
- PID controller design and tuning
- Stability analysis
- State-space control
- Model Predictive Control (MPC)
- Practical controller implementation in ROS 2

With sensing (Chapter 2), actuation (Chapter 3), and control (Chapter 4), you'll have the complete foundation for building Physical AI systems that sense, decide, and act intelligently.

---

## 14. Glossary

| Term | Definition |
|------|------------|
| **Actuator** | Device that converts energy into mechanical motion or force |
| **DC Motor** | Electric motor powered by direct current |
| **Brushless DC (BLDC)** | DC motor with electronic commutation (no brushes) |
| **Servo Motor** | Motor with integrated position feedback and controller |
| **Stepper Motor** | Motor that moves in discrete angular steps |
| **Linear Actuator** | Device that produces linear (straight-line) motion |
| **Hydraulic Actuator** | Actuator powered by pressurized fluid (oil) |
| **Pneumatic Actuator** | Actuator powered by compressed air |
| **Soft Actuator** | Actuator made from compliant/flexible materials |
| **Torque** | Rotational force (Nm or lb-ft) |
| **Speed (RPM)** | Rotational velocity in revolutions per minute |
| **Power** | Rate of energy transfer (Watts); P = τ × ω |
| **Efficiency** | Ratio of output power to input power (%) |
| **Bandwidth** | Frequency response capability (Hz) |
| **PWM** | Pulse Width Modulation; digital control of average voltage |
| **Duty Cycle** | Fraction of time signal is ON (0-100%) |
| **H-Bridge** | Circuit for bidirectional motor control |
| **ESC** | Electronic Speed Controller (for BLDC motors) |
| **Encoder** | Sensor that measures rotational position/velocity |
| **PID Controller** | Proportional-Integral-Derivative feedback controller |
| **Gear Ratio** | Ratio of output to input rotation; trades speed for torque |
| **Backlash** | Mechanical play/slack in gear systems |
| **Forward Kinematics** | Computing end-effector pose from joint angles |
| **Inverse Kinematics** | Computing joint angles from desired end-effector pose |
| **DOF** | Degrees of Freedom; independent motion parameters |
| **Workspace** | Set of reachable positions for an end-effector |
| **Motion Profile** | Planned trajectory for position/velocity/acceleration |
| **Trapezoidal Profile** | Motion profile with accel-cruise-decel phases |
| **S-Curve Profile** | Jerk-limited motion profile for smooth motion |
| **ros2_control** | ROS 2 framework for real-time robot control |
| **Hardware Interface** | Abstraction layer between controllers and actuators |
| **JointState** | ROS 2 message containing joint positions/velocities/efforts |
| **JointTrajectory** | ROS 2 message for time-parameterized motion |
| **Series Elastic Actuator (SEA)** | Actuator with spring between motor and output |
| **Backdrivability** | Ability to move actuator by applying external force |
| **Compliance** | Mechanical flexibility; inverse of stiffness |

---

## 15. Exercises / Review Questions

### Conceptual Questions

**Q1.** Define "actuator" in the context of Physical AI. How do actuators complete the perception-action loop?

**Q2.** Compare and contrast electric, hydraulic, pneumatic, and soft actuators. Create a table showing their relative strengths for: force output, precision, speed, efficiency, and safety.

**Q3.** Explain the difference between brushed DC motors and brushless DC motors. Why are BLDC motors preferred for drones?

**Q4.** What is a servo motor? What components are integrated into a servo that make it different from a basic DC motor?

**Q5.** Explain the torque-speed tradeoff in electric motors. How do gear ratios affect this tradeoff?

### Technical Questions

**Q6.** A DC motor has torque constant $k_t = 0.05$ Nm/A. If 10A flows through the motor, what is the output torque?

**Q7.** A motor spins at 3000 RPM under no load. Convert this to rad/s.

**Q8.** A 100:1 gearbox has 90% efficiency. If the input torque is 0.5 Nm, what is the output torque?

**Q9.** Draw a trapezoidal velocity profile for moving 5 radians with v_max = 2 rad/s and a_max = 4 rad/s². Calculate the total motion time.

**Q10.** For a 2-link planar arm with $L_1 = 0.5$m and $L_2 = 0.4$m, use forward kinematics to find the end-effector position when $\theta_1 = 45°$ and $\theta_2 = 30°$.

**Q11.** For the same arm, calculate the joint angles needed to reach position (0.6, 0.3) using inverse kinematics.

### Practical Questions

**Q12.** You are designing a robotic arm for pick-and-place. The arm must lift 5 kg at 1 meter from the base. What minimum torque is required at the base joint? (Assume gravity = 10 m/s²)

**Q13.** A drone motor has KV rating of 1000 RPM/V. On a 4S battery (14.8V), what is the theoretical no-load RPM?

**Q14.** Explain why PWM is preferred over variable DC voltage for motor speed control.

**Q15.** You need to select actuators for a humanoid robot leg. List the key requirements and justify actuator choices for the hip, knee, and ankle joints.

### Coding Exercises

**Q16.** Write a Python function that generates an S-curve motion profile given distance, max velocity, max acceleration, and max jerk.

**Q17.** Modify the DC motor simulation to include a PID position controller. Tune the gains to achieve position control with less than 5% overshoot.

**Q18.** Write a ROS 2 node that subscribes to `/joint_states`, computes forward kinematics for a 2-link arm, and publishes the end-effector position.

### Answers (Selected)

**A6.** $\tau = k_t \times I = 0.05 \times 10 = 0.5$ Nm

**A7.** `ω = (2π × 3000) / 60 = 314.16 rad/s`

**A8.** `τ_out = τ_in × N × η = 0.5 × 100 × 0.9 = 45 Nm`

**A9.**
- Accel time: `t_a = v_max / a_max = 2/4 = 0.5s`
- Accel distance: `d_a = 0.5 × 4 × 0.5² = 0.5 rad`
- Cruise distance: `d_c = 5 - 2 × 0.5 = 4 rad`
- Cruise time: `t_c = 4/2 = 2s`
- **Total time**: `T = 2 × 0.5 + 2 = 3s`

**A10.**
`x = 0.5 × cos(45°) + 0.4 × cos(75°) = 0.354 + 0.104 = 0.458 m`

`y = 0.5 × sin(45°) + 0.4 × sin(75°) = 0.354 + 0.386 = 0.740 m`

**A12.** $\tau = m \times g \times r = 5 \times 10 \times 1 = 50$ Nm (minimum, not including arm weight or safety factor)

**A13.** $RPM = KV \times V = 1000 \times 14.8 = 14,800$ RPM (no-load theoretical)

---

## Additional Resources

### Documentation
- ros2_control: [control.ros.org](https://control.ros.org/)
- ROS 2 control_msgs: [docs.ros.org](https://docs.ros.org/)
- Gazebo ros2_control: [github.com/ros-controls](https://github.com/ros-controls)

### Books
- *Electric Motors and Drives* by Austin Hughes
- *Introduction to Robotics* by John Craig (kinematics)
- *Modern Robotics* by Lynch & Park (free online)

### Motor Resources
- Motor sizing calculators: Oriental Motor, Maxon
- BLDC fundamentals: Texas Instruments application notes
- Servo selection guides: Dynamixel, Robotis

### Simulation
- Gazebo: [gazebosim.org](https://gazebosim.org/)
- NVIDIA Isaac Sim: [developer.nvidia.com/isaac-sim](https://developer.nvidia.com/isaac-sim)
- PyBullet: Physics simulation for robotics

---

**STATUS: Chapter 3 Completed 100% — Ready for Publishing**

---

*Previous: [Chapter 2: Sensors in Physical AI](./02-sensors.md)*

*Next: [Chapter 4: Control Systems](./04-control-systems.md)*
