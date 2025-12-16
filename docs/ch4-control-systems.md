---
id: ch4-control-systems
title: "Chapter 4: Control Systems in Physical AI"
sidebar_label: "4. Control Systems"
sidebar_position: 4
description: "Core technical chapter covering feedback control, PID controllers, stability analysis, tuning strategies, and ROS 2 control integration for Physical AI systems."
keywords: [control systems, pid, feedback control, stability, ros2_control, robotics, closed-loop]
---

# Chapter 4: Control Systems in Physical AI

> *"Control is the art of using feedback to transform the uncertain into the predictable."*

---

## 1. Overview / Introduction

In Chapters 1-3, we established the perception-action loop: sensors measure the world, actuators affect it. But how does a robot decide *what* to do with sensor measurements? How does it ensure actuators achieve the *desired* behavior despite disturbances, noise, and model uncertainties?

The answer is **control systems**—the algorithms and architectures that close the loop between sensing and acting, using feedback to achieve desired behaviors reliably and robustly.

This chapter introduces the fundamental concepts of feedback control, focusing on the ubiquitous **PID controller**. By the end, you'll understand why feedback is essential, how to design and tune PID controllers, and how to implement them in ROS 2 for real robotic applications.

### What You Will Learn

By the end of this chapter, you will be able to:

- **Define control systems** and explain their role in Physical AI
- **Distinguish open-loop from closed-loop** control with concrete examples
- **Explain feedback loops** and identify all components
- **Understand PID control** and each term's effect on system behavior
- **Analyze stability** and identify when systems become unstable
- **Tune PID controllers** using manual and systematic methods
- **Understand transfer functions** and frequency response (conceptually)
- **Implement PID controllers** in Python and ROS 2
- **Apply filtering techniques** for robust control
- **Configure ros2_control** for feedback control loops

### Prerequisites

| Concept | Level | Notes |
|---------|-------|-------|
| Chapter 1: Physical AI Introduction | **Required** | Perception-action loop |
| Chapter 2: Sensors | **Required** | Feedback, encoders, noise |
| Chapter 3: Actuators | **Required** | Motor control, PWM, dynamics |
| Basic Calculus | Recommended | Derivatives, integrals (explained) |
| Python Programming | Recommended | For code examples |
| ROS 2 Basics | **Required** | Topics, nodes, parameters |

### Chapter Structure

```
1. Overview / Introduction
2. What is Control?
3. Open-Loop vs Closed-Loop Control
4. Feedback System Components
5. Performance Metrics
6. PID Control
7. PID Tuning Strategies
8. Stability Analysis
9. Transfer Functions & Frequency Response (Introduction)
10. Filtering for Control
11. ros2_control Integration
12. Python & ROS 2 Code Examples
13. Real-World Control Applications
14. Mini-Projects
15. Chapter Summary
16. Glossary
17. Exercises / Review Questions
```

---

## 2. What is Control?

### Definition

A **control system** is a system that uses feedback (or feedforward information) to regulate behavior and achieve desired states despite disturbances and uncertainties.

More formally:

> **Control System**: An interconnection of components that manages, commands, directs, or regulates the behavior of a dynamical system to achieve desired objectives.

### The Goal of Control

In Physical AI, control answers the question:

> **"Given what I want (setpoint) and what I observe (sensor measurement), what should I command (actuator input) to make reality match my desire?"**

Control systems bridge the gap between:
- **Desired state**: Where we want the robot to be
- **Actual state**: Where the robot actually is
- **Control action**: What we command to reduce the difference

### Why Control Matters for Physical AI

Physical AI systems face three fundamental challenges that make control essential:

1. **Uncertainty**: The real world is unpredictable. Wind pushes drones, friction varies, objects slip.

2. **Disturbances**: External forces affect the system. A person bumps the robot, terrain changes, loads vary.

3. **Model Errors**: Our mathematical models of robots are imperfect. Friction is hard to model exactly, masses are approximate.

**Feedback control** addresses all three by continuously measuring the actual state and adjusting commands to correct errors in real-time.

---

## 3. Open-Loop vs Closed-Loop Control

### 3.1 Open-Loop Control

**Definition**: Control without feedback—commands are issued based solely on the desired outcome, without measuring the actual result.

```
            OPEN-LOOP CONTROL

    Desired    ┌────────────┐   Control   ┌───────────┐   Actual
    State ────▶│ CONTROLLER │────────────▶│   PLANT   │────────▶ State
               │            │   Command   │ (System)  │
               └────────────┘             └───────────┘

    No feedback path — controller doesn't know actual state!
```

**Examples**:

| System | Description | Open-Loop Because... |
|--------|-------------|---------------------|
| Microwave timer | Set time, runs that long | No temperature sensing |
| Sprinkler timer | Waters at set intervals | No soil moisture sensing |
| Stepper motor (basic) | Sends step pulses | Assumes each step completes |
| Pre-recorded trajectory | Plays back commands | No tracking verification |

**Advantages**:
- Simple (no sensors needed)
- Low cost
- Deterministic behavior
- No stability concerns

**Disadvantages**:
- No error correction
- Sensitive to disturbances
- Requires accurate model
- Cannot compensate for variations

### 3.2 Closed-Loop Control

**Definition**: Control with feedback—commands are adjusted based on the measured difference between desired and actual states.

```
            CLOSED-LOOP (FEEDBACK) CONTROL

    Desired    ┌────────────┐   Control   ┌───────────┐   Actual
    State ────▶│ CONTROLLER │────────────▶│   PLANT   │────────▶ State
       +       │            │   Command   │ (System)  │      │
       │       └────────────┘             └───────────┘      │
       │             ▲                                       │
       │             │ Error                                 │
       │        ┌────┴─────┐                                │
       └────────│ SUBTRACT │◀───────────────────────────────┘
         -      │    (-)   │         Measured State
                └──────────┘               │
                      ▲                    │
                      │              ┌─────┴─────┐
                      └──────────────│  SENSOR   │
                                     └───────────┘
```

**Examples**:

| System | Description | Closed-Loop Because... |
|--------|-------------|----------------------|
| Thermostat | Adjusts heating based on temperature | Measures actual temperature |
| Cruise control | Adjusts throttle for constant speed | Measures actual speed |
| Robot arm | Tracks desired joint angles | Reads encoder positions |
| Drone altitude | Adjusts thrust to maintain height | Uses barometer/sonar |

**Advantages**:
- Error correction (reduces steady-state error)
- Disturbance rejection (handles external forces)
- Robustness to model uncertainty
- Adapts to changing conditions

**Disadvantages**:
- Requires sensors
- Can become unstable if poorly designed
- More complex
- Computational overhead

### 3.3 Comparison Summary

| Aspect | Open-Loop | Closed-Loop |
|--------|-----------|-------------|
| Feedback | None | Continuous |
| Error correction | None | Automatic |
| Disturbance rejection | None | Yes |
| Model dependency | High | Lower |
| Stability risk | None | Yes (if poorly tuned) |
| Complexity | Low | Higher |
| Cost | Lower | Higher (sensors) |
| Robustness | Low | High |

### Key Insight

> **For Physical AI, closed-loop control is almost always essential.** The real world is too uncertain, too noisy, and too variable for open-loop control to work reliably in safety-critical or precision applications.

---

## 4. Feedback System Components

A complete feedback control system has six essential components:

```
                    FEEDBACK CONTROL BLOCK DIAGRAM

                                                    Disturbance (d)
                                                          │
                                                          ▼
    Reference  ┌────┐  Error  ┌────────────┐  Control  ┌──┴──────┐  Output
    (r) ──────▶│ +  │────────▶│ CONTROLLER │─────────▶│  PLANT  │────────▶ (y)
               │  - │    (e)  │   (C)      │   (u)    │  (G)    │    │
               └────┘         └────────────┘          └─────────┘    │
                 ▲                                                    │
                 │                                                    │
                 │              ┌────────────┐                       │
                 └──────────────│   SENSOR   │◀──────────────────────┘
                    Feedback    │   (H)      │   Measurement (y_m)
                                └────────────┘

    Components:
    • r = Reference (setpoint, desired value)
    • e = Error = r - y_m (what we want minus what we measure)
    • C = Controller (e.g., PID, computes control action)
    • u = Control signal (command to actuator)
    • G = Plant (the physical system being controlled)
    • y = Output (actual system state)
    • H = Sensor (measures output)
    • y_m = Measured output (sensor reading)
    • d = Disturbance (external forces, noise)
```

### 4.1 Reference (Setpoint)

**What it is**: The desired state—where we want the system to be.

**Examples**:
- Desired joint angle: 45°
- Desired temperature: 20°C
- Desired drone altitude: 10 meters
- Desired robot position: (x=1.5, y=2.0)

**In ROS 2**: Often comes from trajectory planners, user commands, or higher-level controllers.

### 4.2 Error Signal

**What it is**: The difference between desired and measured states.

$$e(t) = r(t) - y_m(t)$$

**Why it matters**: Error is the controller's input. If error is zero, no correction needed. If error is large, strong correction required.

**Types of error**:
- **Positive error**: Actual is below setpoint (e.g., need to heat)
- **Negative error**: Actual is above setpoint (e.g., need to cool)
- **Zero error**: Perfect tracking (goal achieved)

### 4.3 Controller

**What it is**: The algorithm that computes control action from error.

**Types of controllers**:

| Controller | Description | Complexity |
|------------|-------------|------------|
| P (Proportional) | Output ∝ error | Simplest |
| PI | P + integral of error | Medium |
| PID | PI + derivative of error | Most common |
| State Feedback | Uses full state vector | Advanced |
| MPC | Optimizes over horizon | Complex |

### 4.4 Actuator (Implicit in Plant)

**What it is**: The device that applies control action to the physical system.

**Examples**: Motors, valves, thrusters, heaters

**In diagrams**: Often combined with plant as "controlled system"

### 4.5 Plant (System)

**What it is**: The physical system being controlled—the "thing" we want to regulate.

**Examples**:
- Robot arm (dynamics, inertia, friction)
- Drone (aerodynamics, mass, thrust)
- Temperature (thermal dynamics, heat transfer)
- Vehicle (engine, drivetrain, mass)

**Mathematical model**: Often represented as differential equations or transfer functions.

### 4.6 Sensor

**What it is**: The device that measures the actual output.

**Examples**: Encoders, IMUs, thermometers, cameras, load cells

**Important**: Sensors introduce noise and delays that affect control performance.

---

## 5. Performance Metrics

How do we evaluate if a control system is "good"? We use standard **performance metrics** measured from the step response.

### Step Response

A **step response** is the system's output when the reference suddenly changes from 0 to some constant value.

```
            STEP RESPONSE WITH PERFORMANCE METRICS

    Output (y)
        │
        │                    ┌─── Overshoot (Mp)
        │                   ╱│
    1.0+│   ╭─────────╮───╱──┼──────────────────────
    Mp  │  ╱           ╲╱    │  Settling band (±2%)
    ────┼─╱──────────────────┼──────────────────────
        │╱    ╭──────────────┼──────────────────────
    0.9+│    ╱               │
        │   ╱                │
        │  ╱                 │
        │ ╱                  │
    0.1+│╱                   │
        │────────────────────┼──────────────────────── Time
        0   t_r         t_s
            │           │
            │           └─── Settling Time (t_s)
            └─── Rise Time (t_r)

    Also measure: Steady-State Error (e_ss)
```

### 5.1 Rise Time (tr)

**Definition**: Time for output to rise from 10% to 90% of final value.

**Affected by**: Proportional gain (Kp) primarily

**Tradeoff**: Faster rise time often means more overshoot

### 5.2 Settling Time (ts)

**Definition**: Time for output to stay within ±2% (or ±5%) of final value.

**Affected by**: Derivative gain (Kd), damping ratio

**Typical targets**: 0.5-5 seconds depending on application

### 5.3 Overshoot (Mp)

**Definition**: Maximum peak value above the setpoint, expressed as percentage.

The formula for overshoot is: `Mp = ((y_max - y_final) / y_final) × 100%`

**Affected by**: Derivative gain (Kd) reduces overshoot

**Typical targets**: &lt;5% for precision, &lt;20% for speed

### 5.4 Steady-State Error (ess)

**Definition**: Long-term error after transients have settled.

The formula is: `e_ss = lim(t→∞) [r(t) - y(t)]`

**Affected by**: Integral gain (Ki) eliminates steady-state error

**Typical targets**: Zero for position control

### 5.5 Metrics Summary

| Metric | Symbol | Typical Target | Primary PID Influence |
|--------|--------|----------------|----------------------|
| Rise Time | $t_r$ | < 1-2 s | Kp (↑ faster) |
| Settling Time | $t_s$ | < 2-5 s | Kd (↑ faster settling) |
| Overshoot | $M_p$ | < 5-20% | Kd (↑ reduces), Kp (↓ reduces) |
| Steady-State Error | e_ss | = 0 | Ki (↑ eliminates) |

---

## 6. PID Control

The **PID controller** is the most widely used control algorithm in robotics and industry. Understanding PID is essential for any Physical AI engineer.

### 6.1 The PID Equation

`u(t) = Kp × e(t) + Ki × ∫e(τ)dτ + Kd × de(t)/dt`

Or in compact form:

`u(t) = Kp × e + Ki × ∫e dt + Kd × ė`

Where:
- $u(t)$ = control output (command to actuator)
- $e(t)$ = error = setpoint - measured value
- $K_p$ = proportional gain
- $K_i$ = integral gain
- $K_d$ = derivative gain

### 6.2 Proportional (P) Term

$$u_P = K_p \cdot e(t)$$

**Function**: Produces output proportional to current error.

**Effect**:
- Larger error → larger correction
- Immediate response to error
- Cannot eliminate steady-state error alone

```
            P-ONLY CONTROL RESPONSE

    Output
        │
    1.0 ├──────────────────────────────────────  Setpoint
        │                    ╭─────────────────
        │                   ╱
        │                  ╱
        │                 ╱
        │                ╱    ← Steady-state error (e_ss)
        │               ╱
        │──────────────╱───────────────────────  Actual
        │             ╱
        │            ╱
        │           ╱
        │          ╱
        │─────────╱
        └────────────────────────────────────── Time

    P-only control leaves steady-state error
    Higher Kp → smaller error but more oscillation
```

**Kp Effects**:

| Kp Value | Rise Time | Overshoot | Steady-State Error | Stability |
|----------|-----------|-----------|-------------------|-----------|
| Too Low | Slow | None | Large | Stable |
| Moderate | Medium | Small | Medium | Stable |
| Too High | Fast | Large | Small | Oscillatory |

### 6.3 Integral (I) Term

$$u_I = K_i \cdot \int_0^t e(\tau) d\tau$$

**Function**: Accumulates error over time; produces output based on history.

**Effect**:
- Eliminates steady-state error (integral accumulates until error = 0)
- Responds to persistent errors
- Can cause overshoot and oscillation if too aggressive

```
            INTEGRAL ACTION ELIMINATING OFFSET

    Output                                        Setpoint
        │    ╭────────────────────────────────── 1.0
        │   ╱
        │  ╱
        │ ╱    ← With Ki, output reaches setpoint
        │╱
        ├──────────────────────────────────────
        │
        │                    ← Without Ki, offset remains
        │──────────────────────────────────────
        └────────────────────────────────────── Time

    Integral term "winds up" until error is eliminated
```

**Ki Effects**:

| Ki Value | Rise Time | Overshoot | Steady-State Error | Stability |
|----------|-----------|-----------|-------------------|-----------|
| Zero | Unchanged | Unchanged | Present | Stable |
| Low | Slightly faster | Slightly more | Slow elimination | Stable |
| Moderate | Faster | More | Eliminated | Stable |
| Too High | Fastest | Excessive | Eliminated | Unstable |

**Anti-Windup**: When actuator saturates (hits limits), integral keeps accumulating, causing "windup." Solutions:
- Clamp integral term
- Conditional integration (stop when saturated)
- Back-calculation

### 6.4 Derivative (D) Term

`u_D = Kd × de(t)/dt`

**Function**: Produces output based on rate of change of error.

**Effect**:
- Predicts future error trajectory
- Adds damping, reduces overshoot
- Slows approach to setpoint (prevents overshooting)
- Sensitive to noise (differentiating amplifies high-frequency noise)

```
            DERIVATIVE ACTION (DAMPING)

    Output
        │
    1.0 ├──────────────────────────────────────  Setpoint
        │     Without Kd        With Kd
        │      ╭──╮            ╭───────────────
        │     ╱    ╲          ╱
        │    ╱      ╲        ╱   ← Less overshoot
        │   ╱        ╲      ╱
        │  ╱          ╲    ╱
        │ ╱            ╲  ╱
        │╱              ╲╱
        │
        └────────────────────────────────────── Time

    D term acts as "brake" when approaching setpoint
```

**Kd Effects**:

| Kd Value | Rise Time | Overshoot | Settling Time | Noise Sensitivity |
|----------|-----------|-----------|---------------|-------------------|
| Zero | Fast | High | Long | None |
| Low | Slightly slower | Medium | Medium | Low |
| Moderate | Slower | Low | Short | Medium |
| Too High | Very slow | None | Very long | High |

**Derivative Filtering**: To reduce noise sensitivity:
`u_D = Kd × (N / (1 + N × Tf)) × e(s)`

Or in discrete time, apply a low-pass filter to the derivative term.

### 6.5 PID Component Summary

```
            PID CONTROL BLOCK DIAGRAM

                              ┌─────────────┐
                         ┌───▶│   Kp × e    │───┐
                         │    └─────────────┘   │
                         │                      │
    Error    ┌───────────┼───▶┌─────────────┐   │
    (e) ────▶│  Split    │    │ Ki × ∫e dt  │───┼───▶ Control
             │           │    └─────────────┘   │     Output (u)
             └───────────┤                      │
                         │    ┌─────────────┐   │
                         └───▶│ Kd × de/dt  │───┘
                              └─────────────┘

    P: Reacts to present error (proportional)
    I: Reacts to past error (accumulated history)
    D: Reacts to future error (rate of change)
```

### 6.6 Discrete-Time PID

For digital implementation (computers, microcontrollers), PID must be discretized:

`u[k] = Kp × e[k] + Ki × Ts × Σe[i] + Kd × (e[k] - e[k-1]) / Ts`

Where:
- $T_s$ = sampling period (seconds)
- $k$ = current time step
- $e[k]$ = error at current step
- $e[k-1]$ = error at previous step

**Implementation considerations**:
- Integral accumulation: `integral += e * Ts`
- Derivative approximation: `derivative = (e - e_prev) / Ts`
- Output clamping: Limit `u` to actuator range
- Anti-windup: Stop integral when saturated

---

## 7. PID Tuning Strategies

Selecting appropriate $K_p$, $K_i$, $K_d$ values is called **tuning**. Poor tuning leads to slow response, excessive overshoot, or instability.

### 7.1 Manual Tuning

A systematic approach:

```
MANUAL PID TUNING PROCEDURE

Step 1: Start with Ki = 0, Kd = 0
        ├─ Increase Kp until response is fast but acceptable overshoot
        └─ Observe: Too slow? Increase Kp. Oscillating? Reduce Kp.

Step 2: Add derivative (Kd)
        ├─ Increase Kd to reduce overshoot and oscillation
        └─ Observe: Still overshooting? Increase Kd. Too sluggish? Reduce Kd.

Step 3: Add integral (Ki)
        ├─ Increase Ki to eliminate steady-state error
        └─ Observe: Offset remaining? Increase Ki. Oscillating? Reduce Ki.

Step 4: Fine-tune all three
        └─ Iterate until performance targets met
```

**Rules of Thumb**:

| If you see... | Try... |
|---------------|--------|
| Slow response | Increase Kp |
| Large overshoot | Increase Kd or decrease Kp |
| Oscillation | Decrease Kp, increase Kd |
| Steady-state error | Increase Ki |
| Instability | Decrease all gains |

### 7.2 Ziegler-Nichols Method

A classical tuning method based on finding the **ultimate gain** and **ultimate period**.

**Procedure**:

1. Set $K_i = 0$ and $K_d = 0$
2. Increase $K_p$ until the system oscillates with constant amplitude
3. Record:
   - $K_u$ = ultimate gain (value of $K_p$ at oscillation onset)
   - $P_u$ = ultimate period (oscillation period in seconds)
4. Calculate PID gains from table:

| Controller | $K_p$ | $K_i$ | $K_d$ |
|------------|-------|-------|-------|
| P | 0.5 $K_u$ | — | — |
| PI | 0.45 $K_u$ | 0.54 $K_u / P_u$ | — |
| PID | 0.6 $K_u$ | 1.2 $K_u / P_u$ | 0.075 $K_u \cdot P_u$ |

**Example**:
- Ultimate gain: $K_u = 10$
- Ultimate period: $P_u = 0.5$ s
- PID gains: $K_p = 6$, $K_i = 24$, $K_d = 0.375$

**Limitations**:
- Aggressive tuning (may need refinement)
- Requires inducing oscillation (may not be safe)
- Assumes linear system

### 7.3 Software Auto-Tuning

Modern approaches use optimization:

| Method | Description | Pros | Cons |
|--------|-------------|------|------|
| Relay Feedback | Automatically finds Ku, Pu | Systematic | Still aggressive |
| Optimization | Minimizes cost function | Optimal | Requires model |
| Adaptive | Real-time adjustment | Self-correcting | Complex |
| Machine Learning | Learns from data | Handles nonlinearity | Data hungry |

### 7.4 Tuning Tradeoffs

| Tradeoff | Fast Response | Slow Response |
|----------|---------------|---------------|
| Stability | Less stable | More stable |
| Overshoot | More | Less |
| Energy use | Higher | Lower |
| Robustness | Lower | Higher |

**Practical advice**: Start conservative (low gains), then increase until performance is acceptable. It's easier to speed up a stable system than to stabilize an oscillating one.

---

## 8. Stability Analysis

### 8.1 What is Stability?

A control system is **stable** if bounded inputs produce bounded outputs (BIBO stability).

**Stable**: Output settles to steady state after disturbances
**Marginally Stable**: Output oscillates forever without growing or shrinking
**Unstable**: Output grows without bound

```
            STABILITY TYPES (STEP RESPONSE)

    STABLE                  MARGINALLY STABLE         UNSTABLE

    │ ╭───────────         │ ╭─╮ ╭─╮ ╭─╮            │       ╱
    │╱                     │╱   ╲╱   ╲╱             │      ╱
    ├──────────────        ├──────────────         │     ╱
    │                      │                        │    ╱
    │                      │                        │   ╱
    └─────────────         └─────────────          │  ╱
                                                    │ ╱
    Settles to final       Constant oscillation    │╱
    value                                          └──────────
                                                   Grows forever
```

### 8.2 Causes of Instability

| Cause | Description | Solution |
|-------|-------------|----------|
| Excessive gain | Too much amplification | Reduce Kp, Ki |
| Time delays | Feedback arrives late | Reduce gains, predictive control |
| Positive feedback | Error amplified | Check sensor polarity |
| Actuator saturation | Nonlinearity | Anti-windup, lower gains |
| Resonance | Excites natural frequency | Add filtering, avoid frequency |

### 8.3 Qualitative Stability Indicators

**From step response**:
- Decaying oscillations → Stable
- Constant oscillations → Marginally stable
- Growing oscillations → Unstable
- Exponential growth → Unstable

**From Bode plot** (advanced):
- Phase margin > 45° → Stable
- Gain margin > 6 dB → Stable
- Phase margin ≈ 0° → Marginally stable

### 8.4 Damping Ratio

The **damping ratio** (ζ) characterizes oscillation behavior:

| ζ Value | System Type | Behavior |
|---------|-------------|----------|
| ζ = 0 | Undamped | Infinite oscillation |
| 0 < ζ < 1 | Underdamped | Decaying oscillation |
| ζ = 1 | Critically damped | Fastest non-oscillating |
| ζ > 1 | Overdamped | Slow, no oscillation |

**For PID systems**: Kd increases effective damping.

---

## 9. Transfer Functions & Frequency Response (Introduction)

### 9.1 Transfer Functions

A **transfer function** G(s) relates input to output in the Laplace domain:

`G(s) = Y(s) / U(s)`

Where $s$ is the complex frequency variable.

**Example**: First-order system

`G(s) = K / (τs + 1)`

- $K$ = steady-state gain
- $\tau$ = time constant

**Why transfer functions?**
- Simplify analysis of linear systems
- Enable frequency response analysis
- Standard way to describe system dynamics

### 9.2 PID Transfer Function

The PID controller has transfer function:

`C(s) = Kp + Ki/s + Kd×s`

Or in parallel form:

`C(s) = Kp × (1 + 1/(Ti×s) + Td×s)`

Where $T_i = K_p/K_i$ and $T_d = K_d/K_p$.

### 9.3 Frequency Response (Bode Plots)

A **Bode plot** shows how a system responds to sinusoidal inputs at different frequencies.

```
            BODE PLOT (SIMPLIFIED)

    Magnitude (dB)
        │
    20  ┤────╮
        │     ╲
     0  ┤      ╲
        │       ╲
   -20  ┤        ╲─────────────────
        │
        └────────┬────────┬────────── Frequency (log scale)
               0.1       1        10

    Phase (degrees)
        │
     0° ┤────╮
        │     ╲
   -45° ┤      ╲
        │       ╲
   -90° ┤        ╲─────────────────
        │
        └────────┬────────┬────────── Frequency (log scale)
               0.1       1        10

    Shows gain and phase shift vs. frequency
```

**Key concepts**:
- **Gain margin**: How much gain can increase before instability
- **Phase margin**: How much phase lag before instability
- Higher margins → more robust stability

### 9.4 Practical Interpretation

For most Physical AI applications, you don't need deep Laplace analysis. Key takeaways:

1. **Higher bandwidth** → faster response but more noise sensitivity
2. **Lower bandwidth** → smoother but slower
3. **Phase lag from delays** → reduces stability
4. **PID adds phase lead** (D term) to improve stability margins

---

## 10. Filtering for Control

### 10.1 Why Filter?

Sensor signals contain **noise**—high-frequency disturbances that can destabilize control, especially the derivative term.

### 10.2 Low-Pass Filter

Removes high-frequency noise while preserving the signal:

`y_filtered[k] = α × x[k] + (1-α) × y_filtered[k-1]`

Where `α = Ts / (Ts + τf)` and τf is the filter time constant.

```
            LOW-PASS FILTERING

    Raw Signal (with noise)          Filtered Signal

      ● ●                               ●●●
     ●   ●●   ●                        ●   ●●
    ●      ●●●  ●●                    ●      ●●
                 ●  ●  ●                      ●●
                   ●● ●●                        ●●
                      ●●                          ●

    Noise removed, signal trend preserved
```

### 10.3 Derivative Filtering

The D term is especially noise-sensitive. Common solution:

`D_filtered = (Kd × s) / (1 + τd × s)`

In discrete time:
```python
derivative_filtered = alpha_d * derivative + (1 - alpha_d) * derivative_prev
```

### 10.4 Complementary Filter for Control

Often used with IMUs (combining gyroscope and accelerometer):

`θ = α × (θ_prev + ω × dt) + (1-α) × θ_accel`

This filters high-frequency noise from accelerometer while preventing gyroscope drift.

---

## 11. ros2_control Integration

### 11.1 ros2_control Architecture

```
            ros2_control ARCHITECTURE FOR PID

    ┌─────────────────────────────────────────────────────────┐
    │                  USER APPLICATION                       │
    │     (Trajectory planning, task commands)                │
    └───────────────────────┬─────────────────────────────────┘
                            │ Reference/Setpoint
                            ▼
    ┌─────────────────────────────────────────────────────────┐
    │                 CONTROLLER MANAGER                       │
    │  ┌─────────────────────────────────────────────────┐   │
    │  │      PID POSITION CONTROLLER                     │   │
    │  │  ┌─────────┐  ┌─────────┐  ┌─────────┐         │   │
    │  │  │    P    │  │    I    │  │    D    │         │   │
    │  │  └────┬────┘  └────┬────┘  └────┬────┘         │   │
    │  │       └───────────┬┴───────────┘               │   │
    │  │                   ▼                             │   │
    │  │           Control Output (u)                    │   │
    │  └─────────────────────────────────────────────────┘   │
    └───────────────────────┬─────────────────────────────────┘
                            │
                            ▼
    ┌─────────────────────────────────────────────────────────┐
    │                  HARDWARE INTERFACE                      │
    │         Command Interface    State Interface             │
    │             (effort)          (position)                 │
    └───────────────────────┬─────────────────────────────────┘
                            │
                ┌───────────┴───────────┐
                ▼                       ▼
         ┌───────────┐           ┌───────────┐
         │  Gazebo   │           │   Real    │
         │    Sim    │           │  Hardware │
         └───────────┘           └───────────┘
```

### 11.2 Controller Configuration (YAML)

```yaml
# pid_controller_config.yaml
controller_manager:
  ros__parameters:
    update_rate: 500  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    position_controller:
      type: position_controllers/JointGroupPositionController

position_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3

    # PID gains per joint
    gains:
      joint_1:
        p: 100.0
        i: 10.0
        d: 5.0
        i_clamp: 50.0  # Anti-windup
      joint_2:
        p: 80.0
        i: 8.0
        d: 4.0
        i_clamp: 40.0
      joint_3:
        p: 50.0
        i: 5.0
        d: 2.5
        i_clamp: 25.0

    state_publish_rate: 100.0
```

### 11.3 Launching Controllers

```bash
# Load and configure controller
ros2 control load_controller position_controller

# Start controller
ros2 control switch_controllers --start position_controller

# Check status
ros2 control list_controllers
```

---

## 12. Python & ROS 2 Code Examples

### 12.1 Basic PID Controller (Pure Python)

```python
"""
PID Controller Class - Pure Python Implementation
"""

class PIDController:
    """
    A complete PID controller with anti-windup and derivative filtering.
    """

    def __init__(self, kp, ki, kd, output_min=-float('inf'),
                 output_max=float('inf'), integral_max=None,
                 derivative_filter=0.1):
        """
        Initialize PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_min: Minimum output value
            output_max: Maximum output value
            integral_max: Maximum integral accumulation (anti-windup)
            derivative_filter: Low-pass filter coefficient for derivative (0-1)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_max = integral_max or float('inf')
        self.derivative_filter = derivative_filter

        # Internal state
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.prev_time = None

    def update(self, setpoint, measurement, current_time):
        """
        Compute PID control output.

        Args:
            setpoint: Desired value
            measurement: Current measured value
            current_time: Current time in seconds

        Returns:
            Control output (clamped to output limits)
        """
        # Calculate error
        error = setpoint - measurement

        # Calculate time delta
        if self.prev_time is None:
            dt = 0.01  # Default for first iteration
        else:
            dt = current_time - self.prev_time
            if dt <= 0:
                dt = 0.01

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(-self.integral_max,
                           min(self.integral_max, self.integral))
        i_term = self.ki * self.integral

        # Derivative term with filtering
        if dt > 0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0

        # Apply low-pass filter to derivative
        filtered_derivative = (self.derivative_filter * derivative +
                              (1 - self.derivative_filter) * self.prev_derivative)
        d_term = self.kd * filtered_derivative

        # Calculate total output
        output = p_term + i_term + d_term

        # Clamp output
        output = max(self.output_min, min(self.output_max, output))

        # Anti-windup: if output is saturated, don't accumulate integral
        if output == self.output_max or output == self.output_min:
            self.integral -= error * dt  # Undo integral accumulation

        # Store state for next iteration
        self.prev_error = error
        self.prev_derivative = filtered_derivative
        self.prev_time = current_time

        return output

    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.prev_time = None


# Simulation example
def simulate_pid_control():
    """Simulate PID control of a first-order system."""
    import math

    # System parameters (first-order: tau * dy/dt + y = K * u)
    K = 1.0      # System gain
    tau = 0.5    # Time constant
    dt = 0.01    # Simulation time step

    # PID controller
    pid = PIDController(kp=2.0, ki=1.0, kd=0.5,
                       output_min=-10, output_max=10)

    # Simulation
    setpoint = 1.0
    y = 0.0  # Initial output
    time_data = []
    setpoint_data = []
    output_data = []
    control_data = []

    print("=== PID Control Simulation ===")
    print(f"Setpoint: {setpoint}")
    print(f"Gains: Kp={pid.kp}, Ki={pid.ki}, Kd={pid.kd}")
    print()
    print(f"{'Time':>6} | {'Setpoint':>8} | {'Output':>8} | {'Error':>8} | {'Control':>8}")
    print("-" * 55)

    t = 0
    while t < 5.0:
        # PID control
        u = pid.update(setpoint, y, t)

        # System dynamics (first-order)
        dy_dt = (K * u - y) / tau
        y += dy_dt * dt

        # Store data
        time_data.append(t)
        setpoint_data.append(setpoint)
        output_data.append(y)
        control_data.append(u)

        # Print every 0.5 seconds
        if abs(t % 0.5) < dt:
            error = setpoint - y
            print(f"{t:6.2f} | {setpoint:8.3f} | {y:8.3f} | {error:8.3f} | {u:8.3f}")

        t += dt

    # Calculate metrics
    final_error = abs(setpoint - output_data[-1])
    max_output = max(output_data)
    overshoot = (max_output - setpoint) / setpoint * 100 if setpoint != 0 else 0

    print("-" * 55)
    print(f"Final error: {final_error:.4f}")
    print(f"Max overshoot: {overshoot:.1f}%")

    return time_data, setpoint_data, output_data, control_data


if __name__ == "__main__":
    simulate_pid_control()
```

### 12.2 PID Step Response Comparison

```python
"""
Compare P, PI, and PID Control Step Responses
"""

def compare_controllers():
    """Compare different controller configurations."""

    # System: first-order with delay
    K = 1.0
    tau = 0.5
    dt = 0.01
    setpoint = 1.0
    duration = 6.0

    configs = [
        ("P-only", 2.0, 0.0, 0.0),
        ("PI", 2.0, 1.0, 0.0),
        ("PID", 2.0, 1.0, 0.5),
        ("Aggressive PID", 5.0, 2.0, 1.0),
    ]

    print("=== Controller Comparison ===")
    print(f"{'Controller':<15} | {'Rise Time':>10} | {'Overshoot':>10} | {'SS Error':>10}")
    print("-" * 55)

    for name, kp, ki, kd in configs:
        # Create controller
        pid = PIDController(kp, ki, kd, output_min=-10, output_max=10)

        # Simulate
        y = 0.0
        t = 0
        outputs = []
        rise_time = None
        settled = False

        while t < duration:
            u = pid.update(setpoint, y, t)
            dy_dt = (K * u - y) / tau
            y += dy_dt * dt
            outputs.append(y)

            # Check rise time (first time above 90%)
            if rise_time is None and y >= 0.9 * setpoint:
                rise_time = t

            t += dt

        # Calculate metrics
        max_val = max(outputs)
        overshoot = max(0, (max_val - setpoint) / setpoint * 100)
        ss_error = abs(setpoint - outputs[-1])

        rise_str = f"{rise_time:.2f}s" if rise_time else "N/A"
        print(f"{name:<15} | {rise_str:>10} | {overshoot:>9.1f}% | {ss_error:>10.4f}")


if __name__ == "__main__":
    compare_controllers()
```

### 12.3 ROS 2 PID Controller Node

```python
#!/usr/bin/env python3
"""
ROS 2 PID Controller Node for Joint Position Control
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math


class PIDControllerNode(Node):
    """ROS 2 node implementing PID control for joint positions."""

    def __init__(self):
        super().__init__('pid_controller_node')

        # Declare parameters
        self.declare_parameter('kp', 10.0)
        self.declare_parameter('ki', 1.0)
        self.declare_parameter('kd', 0.5)
        self.declare_parameter('control_rate', 100.0)  # Hz

        # Get parameters
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        control_rate = self.get_parameter('control_rate').value

        # Controller state (for each joint)
        self.integral = {}
        self.prev_error = {}
        self.prev_time = None

        # Setpoint
        self.setpoint = {}

        # Publishers and subscribers
        self.command_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_effort_controller/commands',
            10
        )

        self.state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.setpoint_sub = self.create_subscription(
            JointState,
            '/desired_joint_states',
            self.setpoint_callback,
            10
        )

        # Control loop timer
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)

        # Current state
        self.current_positions = {}
        self.joint_names = []

        self.get_logger().info(
            f'PID Controller started: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}'
        )

    def setpoint_callback(self, msg):
        """Update setpoints from desired joint states."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.setpoint[name] = msg.position[i]

    def joint_state_callback(self, msg):
        """Update current joint positions."""
        self.joint_names = list(msg.name)
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_positions[name] = msg.position[i]

                # Initialize controller state if needed
                if name not in self.integral:
                    self.integral[name] = 0.0
                    self.prev_error[name] = 0.0

    def control_loop(self):
        """Main PID control loop - called at control_rate Hz."""
        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time
        if dt <= 0:
            return

        commands = []

        for name in self.joint_names:
            # Get setpoint and current position
            sp = self.setpoint.get(name, 0.0)
            pos = self.current_positions.get(name, 0.0)

            # Calculate error
            error = sp - pos

            # PID computation
            # Proportional
            p_term = self.kp * error

            # Integral
            self.integral[name] += error * dt
            # Anti-windup
            self.integral[name] = max(-10.0, min(10.0, self.integral[name]))
            i_term = self.ki * self.integral[name]

            # Derivative
            d_term = self.kd * (error - self.prev_error.get(name, 0.0)) / dt

            # Total output
            output = p_term + i_term + d_term

            # Clamp output
            output = max(-100.0, min(100.0, output))

            commands.append(output)
            self.prev_error[name] = error

        # Publish command
        if commands:
            msg = Float64MultiArray()
            msg.data = commands
            self.command_pub.publish(msg)

        self.prev_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()

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

### 12.4 Ziegler-Nichols Tuning Helper

```python
"""
Ziegler-Nichols PID Tuning Calculator
"""

def ziegler_nichols_tuning(ku, pu, controller_type='PID'):
    """
    Calculate PID gains using Ziegler-Nichols method.

    Args:
        ku: Ultimate gain (gain at oscillation onset)
        pu: Ultimate period (oscillation period in seconds)
        controller_type: 'P', 'PI', or 'PID'

    Returns:
        Dictionary with Kp, Ki, Kd gains
    """
    if controller_type == 'P':
        kp = 0.5 * ku
        ki = 0.0
        kd = 0.0
    elif controller_type == 'PI':
        kp = 0.45 * ku
        ki = 0.54 * ku / pu
        kd = 0.0
    elif controller_type == 'PID':
        kp = 0.6 * ku
        ki = 1.2 * ku / pu
        kd = 0.075 * ku * pu
    else:
        raise ValueError(f"Unknown controller type: {controller_type}")

    return {'Kp': kp, 'Ki': ki, 'Kd': kd}


def print_tuning_table(ku, pu):
    """Print Ziegler-Nichols tuning table."""
    print("=== Ziegler-Nichols Tuning ===")
    print(f"Ultimate Gain (Ku): {ku}")
    print(f"Ultimate Period (Pu): {pu} seconds")
    print()
    print(f"{'Controller':<10} | {'Kp':>10} | {'Ki':>10} | {'Kd':>10}")
    print("-" * 50)

    for ctrl_type in ['P', 'PI', 'PID']:
        gains = ziegler_nichols_tuning(ku, pu, ctrl_type)
        print(f"{ctrl_type:<10} | {gains['Kp']:>10.3f} | {gains['Ki']:>10.3f} | {gains['Kd']:>10.3f}")


# Example usage
if __name__ == "__main__":
    # Example: Found Ku=10, Pu=0.5s through oscillation test
    ku = 10.0
    pu = 0.5

    print_tuning_table(ku, pu)

    print()
    print("To use these gains in ROS 2 YAML:")
    gains = ziegler_nichols_tuning(ku, pu, 'PID')
    print(f"""
gains:
  joint_1:
    p: {gains['Kp']:.1f}
    i: {gains['Ki']:.1f}
    d: {gains['Kd']:.3f}
    i_clamp: {gains['Ki'] * 5:.1f}
""")
```

---

## 13. Real-World Control Applications

### 13.1 Robot Arm Joint Control

```
            ROBOT ARM PID CONTROL

    Trajectory     ┌─────────┐   Torque    ┌─────────┐   Angle
    Planner  ─────▶│   PID   │───────────▶│  Motor  │──────────▶
                   │ per     │   Command  │  + Arm  │
                   │ Joint   │            │ Dynamics│
                   └────┬────┘            └────┬────┘
                        ▲                      │
                        │                      │
                   ┌────┴────┐                │
                   │ Encoder │◀───────────────┘
                   └─────────┘

    Typical gains: Kp=100-500, Ki=10-50, Kd=5-20
    Update rate: 500-1000 Hz
```

**Challenges**:
- Varying load (inertia changes with arm configuration)
- Gravity compensation (torque needed just to hold position)
- Coupling between joints

**Solutions**:
- Gain scheduling (adjust gains based on configuration)
- Feedforward gravity compensation
- Computed torque control (advanced)

### 13.2 Quadcopter Attitude Control

```
            QUADCOPTER ATTITUDE CONTROL

                        ┌───────────────────┐
    Desired    ────────▶│   ATTITUDE PID    │
    Attitude            │  (Roll, Pitch,    │
    (φ,θ,ψ)             │     Yaw)          │
                        └─────────┬─────────┘
                                  │
                                  ▼
                        ┌───────────────────┐
                        │   MOTOR MIXING    │
                        │  (Allocate to     │
                        │   4 motors)       │
                        └─────────┬─────────┘
                                  │
                    ┌─────────────┼─────────────┐
                    ▼             ▼             ▼
                 Motor 1      Motor 2       Motor 3,4
                  (M1)         (M2)         (M3,M4)
                    │             │             │
                    └─────────────┼─────────────┘
                                  │
                        ┌─────────┴─────────┐
                        │       IMU         │
                        │ (Gyro + Accel)    │
                        └───────────────────┘

    Update rate: 500-2000 Hz (fast dynamics)
    Typical: Cascaded PID (angle → rate → motor)
```

### 13.3 Balancing Robot (Inverted Pendulum)

```
            BALANCING ROBOT CONTROL

    Upright     ┌──────────┐    Motor     ┌──────────┐
    Setpoint───▶│   PID    │─────────────▶│  Wheels  │
    (θ = 0)     │          │   Command    │  + Body  │
                └────┬─────┘              └────┬─────┘
                     ▲                         │
                     │                         │
                ┌────┴────┐                   │
                │   IMU   │◀──────────────────┘
                │ (Tilt)  │
                └─────────┘

    Challenge: Inherently unstable (pole at origin)
    Requires: Very fast control (>100 Hz)
    Gains: High Kp, moderate Kd (derivative crucial)
```

### 13.4 Comparison of Applications

| Application | Control Rate | Primary Challenge | Critical Term |
|-------------|--------------|-------------------|---------------|
| Robot arm | 500-1000 Hz | Precision, coupling | P (accuracy) |
| Quadcopter | 500-2000 Hz | Fast dynamics, stability | D (stability) |
| Balancing | 100-500 Hz | Inherent instability | D (prevent fall) |
| Mobile base | 50-100 Hz | Path tracking | I (no drift) |
| Gripper force | 100-500 Hz | Contact stability | D (no bounce) |

---

## 14. Mini-Projects

### Project 1: PID Tuning Simulator

**Objective**: Build an interactive PID tuner that visualizes the effect of gains on step response.

**Tasks**:
1. Implement a first-order or second-order plant simulation
2. Create sliders or inputs for Kp, Ki, Kd
3. Plot step response in real-time
4. Display metrics: rise time, overshoot, settling time, steady-state error
5. Implement Ziegler-Nichols auto-tuning

**Deliverable**: Python script with matplotlib visualization

### Project 2: ROS 2 Joint Position Controller

**Objective**: Implement a PID position controller for a simulated robot arm in Gazebo.

**Tasks**:
1. Set up a robot arm URDF with ros2_control
2. Implement PID controller node subscribing to joint states
3. Publish effort commands to control joints
4. Tune gains to achieve:
   - Rise time < 0.5 seconds
   - Overshoot < 10%
   - Steady-state error < 0.01 rad
5. Test trajectory tracking (sine wave, step sequence)

**Deliverable**: ROS 2 package with controller node, launch file, and tuned YAML

### Project 3: Control Performance Analysis

**Objective**: Compare different control strategies on the same task.

**Tasks**:
1. Implement P-only, PI, PD, and PID controllers
2. Run each on the same step response task
3. Log data: setpoint, actual, error, control effort
4. Calculate and compare metrics
5. Create visualization comparing all controllers
6. Write analysis: When is each controller best?

**Deliverable**: Python notebook with analysis and conclusions

---

## 15. Chapter Summary

### Key Takeaways

1. **Control Systems Close the Loop**: Sensors measure, controllers decide, actuators act—control completes the perception-action loop.

2. **Open-Loop vs Closed-Loop**:
   - Open-loop: No feedback, sensitive to disturbances
   - Closed-loop: Uses feedback to correct errors, robust

3. **Feedback Components**:
   - Reference (setpoint)
   - Error (reference - measurement)
   - Controller (computes action)
   - Plant (physical system)
   - Sensor (measures output)

4. **PID Control**:
   - **P (Proportional)**: Responds to current error, faster response
   - **I (Integral)**: Eliminates steady-state error, can cause overshoot
   - **D (Derivative)**: Predicts future, adds damping, noise-sensitive

5. **Performance Metrics**:
   - Rise time: Speed of response
   - Overshoot: Peak beyond setpoint
   - Settling time: Time to stabilize
   - Steady-state error: Long-term accuracy

6. **Stability**: System is stable if outputs remain bounded. High gains, delays, and positive feedback cause instability.

7. **PID Tuning**:
   - Manual: P → D → I progression
   - Ziegler-Nichols: Systematic method using Ku, Pu
   - Trade speed for stability

8. **Practical Implementation**:
   - Use filtering for derivative term
   - Implement anti-windup for integral term
   - ros2_control provides hardware abstraction

### What's Next: Chapter 5 - ROS 2 Fundamentals

In Chapter 5, we dive deep into **ROS 2** (Robot Operating System 2)—the middleware that ties everything together:

- ROS 2 architecture and concepts
- Nodes, topics, services, actions
- Launch files and parameter management
- Building robot packages
- tf2 transforms for robot coordination
- Visualization with RViz
- Practical robot programming workflows

With sensors (Ch. 2), actuators (Ch. 3), and control (Ch. 4), you now understand the components. ROS 2 is the framework that integrates them into complete, production-ready robotic systems.

---

## 16. Glossary

| Term | Definition |
|------|------------|
| **Control System** | System that uses feedback to regulate behavior and achieve desired states |
| **Open-Loop Control** | Control without feedback; commands based only on desired output |
| **Closed-Loop Control** | Control with feedback; commands adjusted based on measured error |
| **Feedback** | Using measured output to adjust control input |
| **Setpoint (Reference)** | Desired value for the controlled variable |
| **Error Signal** | Difference between setpoint and measured output (e = r - y) |
| **Controller** | Algorithm that computes control action from error |
| **Plant** | Physical system being controlled |
| **PID Controller** | Controller using Proportional, Integral, and Derivative terms |
| **Proportional (P)** | Control term proportional to current error |
| **Integral (I)** | Control term proportional to accumulated error history |
| **Derivative (D)** | Control term proportional to rate of change of error |
| **Gain** | Multiplier that determines controller aggressiveness (Kp, Ki, Kd) |
| **Rise Time** | Time for output to reach 90% of setpoint |
| **Settling Time** | Time for output to stay within ±2% of setpoint |
| **Overshoot** | Maximum peak value beyond setpoint (percentage) |
| **Steady-State Error** | Long-term error after transients have settled |
| **Stability** | Property where bounded inputs produce bounded outputs |
| **Instability** | Output grows without bound; system diverges |
| **Damping** | Reduction of oscillations; increased by D term |
| **Anti-Windup** | Mechanism to prevent integral term from accumulating during saturation |
| **Transfer Function** | Ratio of output to input in Laplace domain: G(s) = Y(s)/U(s) |
| **Bandwidth** | Range of frequencies the system can respond to effectively |
| **Phase Margin** | Amount of additional phase lag before instability |
| **Gain Margin** | Amount of additional gain before instability |
| **Bode Plot** | Graph showing magnitude and phase vs. frequency |
| **Ziegler-Nichols** | Classical PID tuning method using ultimate gain and period |
| **Discrete-Time** | Implementation using samples at fixed time intervals |
| **Sampling Rate** | Frequency of controller updates (Hz) |
| **ros2_control** | ROS 2 framework for real-time robot control |
| **Controller Manager** | ROS 2 component that loads and manages controllers |
| **Hardware Interface** | Abstraction layer between controllers and physical hardware |

---

## 17. Exercises / Review Questions

### Conceptual Questions

**Q1.** Define "control system" and explain why feedback is essential for Physical AI applications.

**Q2.** Classify each example as open-loop or closed-loop, and explain why:
- a) A washing machine with a fixed 30-minute cycle
- b) A self-driving car maintaining lane position
- c) A stepper motor moving 100 steps on command
- d) A thermostat maintaining room temperature

**Q3.** Draw a complete feedback control block diagram and label all components: reference, error, controller, actuator, plant, sensor.

**Q4.** Explain the function of each PID term (P, I, D). Which term eliminates steady-state error? Which reduces overshoot?

**Q5.** What is anti-windup and why is it necessary?

### Technical Questions

**Q6.** A PID controller has gains Kp=10, Ki=2, Kd=1. The current error is 5, the integral of error is 3, and the derivative of error is -2. Calculate the control output.

**Q7.** Using Ziegler-Nichols method: If the ultimate gain Ku=20 and ultimate period Pu=0.4 seconds, calculate the PID gains.

**Q8.** A system has rise time of 0.3s, 15% overshoot, and settling time of 1.2s. Is this system underdamped, critically damped, or overdamped? What would you adjust to reduce overshoot?

**Q9.** Explain why the derivative term is sensitive to noise. How do you mitigate this in practice?

**Q10.** A control loop runs at 100 Hz. What is the sampling period Ts? Write the discrete-time PID equation using this sampling period.

### Practical Questions

**Q11.** You are tuning a robot arm joint controller. Initially with P-only control, the system is slow and has steady-state error of 0.1 rad. What steps would you take to improve performance?

**Q12.** Your quadcopter is oscillating when trying to hover. What does this indicate about your PID gains? What should you adjust?

**Q13.** A robot gripper using force control keeps bouncing when it contacts an object. Which PID term should you increase to fix this?

**Q14.** You need to control a system with significant time delay (100ms). Why is this challenging for PID control? What precautions should you take?

**Q15.** Design a control strategy for a mobile robot to follow a line on the floor. Draw the block diagram and identify the reference, sensor, controller, and actuator.

### Coding Questions

**Q16.** Implement a discrete-time PID controller in Python with:
- Anti-windup (integral clamping)
- Derivative filtering (low-pass filter)
- Output saturation

**Q17.** Write a ROS 2 node that subscribes to `/joint_states`, implements PID control for one joint, and publishes effort commands. Include parameter declarations for Kp, Ki, Kd.

**Q18.** Create a simulation that compares P, PI, PD, and PID control on a first-order system. Plot all four step responses on the same graph.

### Answers (Selected)

**A6.**
`u = Kp × e + Ki × ∫e + Kd × ė = 10(5) + 2(3) + 1(-2) = 50 + 6 - 2 = 54`

**A7.**
- $K_p = 0.6 \times 20 = 12$
- $K_i = 1.2 \times 20 / 0.4 = 60$
- $K_d = 0.075 \times 20 \times 0.4 = 0.6$

**A8.** Underdamped (has overshoot and oscillation). Increase Kd or decrease Kp to reduce overshoot.

**A10.** $T_s = 1/100 = 0.01$ seconds

`u[k] = Kp × e[k] + Ki × 0.01 × Σe[i] + Kd × (e[k] - e[k-1]) / 0.01`

**A12.** Oscillation indicates marginal stability. Reduce Kp (primary cause) or increase Kd (add damping).

---

## Additional Resources

### Books
- *Feedback Control of Dynamic Systems* by Franklin, Powell, Emami-Naeini
- *Modern Control Engineering* by Ogata
- *PID Controllers: Theory, Design, and Tuning* by Åström and Hägglund

### Online Resources
- Control Tutorials for MATLAB and Simulink: ctms.engin.umich.edu
- ROS 2 Control documentation: control.ros.org
- PID Tuning Guide: pidtuning.net

### Software Tools
- MATLAB/Simulink Control System Toolbox
- Python Control Systems Library: python-control
- PlotJuggler for ROS 2 data visualization

### Videos
- Brian Douglas Control Systems Lectures (YouTube)
- MATLAB Tech Talks on Control Systems

---

**STATUS: Chapter 4 Completed 100% — Ready for Publishing**

---

*Previous: [Chapter 3: Actuators in Physical AI](./03-actuators.md)*

*Next: [Chapter 5: ROS 2 Fundamentals](./05-ros2-fundamentals.md)*
