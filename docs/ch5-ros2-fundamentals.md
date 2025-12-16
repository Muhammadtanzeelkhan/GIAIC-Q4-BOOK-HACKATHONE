---
id: ch5-ros2-fundamentals
title: "Chapter 5: ROS 2 Fundamentals"
sidebar_label: "5. ROS 2 Fundamentals"
sidebar_position: 5
---

# Chapter 5: ROS 2 Fundamentals

## 5.1 Chapter Overview

The Robot Operating System 2 (ROS 2) represents the culmination of over a decade of robotics middleware development, providing the software infrastructure that transforms individual components—sensors, actuators, and controllers—into cohesive robotic systems. While previous chapters focused on the hardware and control theory foundations, this chapter bridges that knowledge with the software architecture that makes modern Physical AI systems possible.

ROS 2 is not an operating system in the traditional sense but rather a **middleware framework** that provides communication infrastructure, hardware abstraction, package management, and a vast ecosystem of tools and libraries. For Physical AI practitioners, ROS 2 serves as the common language through which perception systems communicate with planners, planners communicate with controllers, and controllers communicate with actuators.

### Learning Objectives

By the end of this chapter, you will be able to:

1. **Understand ROS 2 Architecture**: Explain the node-based computation graph and communication patterns
2. **Master Communication Primitives**: Implement Topics, Services, and Actions for different use cases
3. **Configure Quality of Service**: Apply appropriate QoS profiles for reliable robot communication
4. **Build ROS 2 Packages**: Create, build, and organize Python packages using colcon
5. **Write Publisher/Subscriber Nodes**: Develop complete rclpy applications with proper lifecycle management
6. **Create Launch Files**: Compose complex systems using Python launch files
7. **Work with Parameters**: Configure nodes dynamically using the parameter system
8. **Understand Robot Descriptions**: Parse and utilize URDF files for robot modeling
9. **Use Coordinate Transforms**: Navigate the TF2 transformation system for spatial reasoning
10. **Apply Essential Tools**: Utilize rviz2, ros2 bag, and CLI tools for development and debugging

### Prerequisites

This chapter assumes familiarity with:
- Python programming (functions, classes, decorators)
- Basic Linux/terminal commands
- Concepts from Chapters 1-4 (sensors, actuators, control systems)
- Understanding of coordinate frames and transformations

### Chapter Roadmap

```
┌─────────────────────────────────────────────────────────────────────┐
│                    ROS 2 FUNDAMENTALS ROADMAP                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  [5.2 What is ROS 2?]──────►[5.3 Core Concepts]                    │
│          │                          │                               │
│          ▼                          ▼                               │
│  [5.4 DDS & QoS]◄──────────[5.5 rclpy Basics]                      │
│          │                          │                               │
│          ▼                          ▼                               │
│  [5.6 Packages]─────────►[5.7 Pub/Sub Implementation]              │
│          │                          │                               │
│          ▼                          ▼                               │
│  [5.8 Launch Files]◄────────[5.9 Parameters]                       │
│          │                          │                               │
│          ▼                          ▼                               │
│  [5.10 URDF]──────────────►[5.11 ros2_control]                     │
│          │                          │                               │
│          ▼                          ▼                               │
│  [5.12 Lifecycle]◄─────────[5.13 TF2 Transforms]                   │
│          │                          │                               │
│          └──────────►[5.14 Tools & Visualization]                  │
│                              │                                      │
│                              ▼                                      │
│                    [5.15 Mini-Projects]                             │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 5.2 What is ROS 2?

### 5.2.1 From ROS 1 to ROS 2

The original Robot Operating System (ROS 1), developed at Willow Garage starting in 2007, revolutionized robotics research by providing a common framework for code sharing and collaboration. However, ROS 1 was designed primarily for single-robot research in controlled environments, with limitations that became apparent as robotics moved toward commercial and safety-critical applications.

**ROS 1 Limitations:**
- Single point of failure (rosmaster)
- No real-time support
- Limited security model
- Unreliable network communication
- Python 2 dependency

ROS 2, first released in 2017, was a complete redesign addressing these limitations while maintaining the conceptual model that made ROS 1 successful.

### 5.2.2 Key Improvements in ROS 2

| Aspect | ROS 1 | ROS 2 |
|--------|-------|-------|
| **Discovery** | Centralized (rosmaster) | Distributed (DDS) |
| **Communication** | Custom TCPROS/UDPROS | Industry-standard DDS |
| **Real-time** | Not supported | Real-time capable |
| **Security** | None built-in | DDS Security, SROS2 |
| **Platforms** | Linux only (official) | Linux, Windows, macOS |
| **Python** | Python 2 | Python 3 |
| **Lifecycle** | None | Managed lifecycle nodes |
| **QoS** | Best-effort only | Configurable QoS policies |

### 5.2.3 ROS 2 Distributions

ROS 2 follows a time-based release schedule aligned with Ubuntu LTS releases:

```
Distribution Timeline:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
2020 ──── Foxy Fitzroy (LTS) ──── Ubuntu 20.04 ──── EOL: 2023
2022 ──── Humble Hawksbill (LTS) ──── Ubuntu 22.04 ──── EOL: 2027
2024 ──── Jazzy Jalisco (LTS) ──── Ubuntu 24.04 ──── EOL: 2029
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

This textbook targets **ROS 2 Humble Hawksbill**, the current Long-Term Support (LTS) release with support until 2027.

### 5.2.4 The ROS 2 Ecosystem

```
┌─────────────────────────────────────────────────────────────────────┐
│                      ROS 2 ECOSYSTEM LAYERS                         │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    APPLICATION LAYER                         │   │
│  │   Navigation2 │ MoveIt2 │ Gazebo │ SLAM Toolbox │ Custom    │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              ▲                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    ROBOT LAYER                               │   │
│  │   ros2_control │ robot_state_publisher │ joint_state_pub    │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              ▲                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                 COMMUNICATION LAYER                          │   │
│  │   Topics │ Services │ Actions │ Parameters │ TF2             │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              ▲                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                   MIDDLEWARE LAYER                           │   │
│  │   DDS (Fast-DDS, Cyclone DDS, RTI Connext)                   │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              ▲                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                   OPERATING SYSTEM                           │   │
│  │   Linux │ Windows │ macOS │ RTOS (limited)                   │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 5.3 Core Concepts: Nodes, Topics, Services, and Actions

### 5.3.1 The Computation Graph

ROS 2 applications are structured as a **computation graph**—a network of processes (nodes) connected by communication channels. This architecture enables:

- **Modularity**: Each node performs a specific function
- **Reusability**: Nodes can be combined in different configurations
- **Fault isolation**: Node failures don't crash the entire system
- **Distributed computing**: Nodes can run on different machines

```
┌─────────────────────────────────────────────────────────────────────┐
│                EXAMPLE COMPUTATION GRAPH                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌──────────┐    /scan      ┌──────────┐    /map                  │
│   │  LiDAR   │──────────────►│   SLAM   │──────────►               │
│   │  Driver  │               │   Node   │                          │
│   └──────────┘               └────┬─────┘                          │
│                                   │                                 │
│   ┌──────────┐    /image         │ /odom   ┌──────────┐           │
│   │  Camera  │───────────┐       │         │Navigation│◄──/cmd_vel│
│   │  Driver  │           │       ▼         │  Stack   │───────────►│
│   └──────────┘           │  ┌────────┐     └──────────┘           │
│                          └─►│Fusion  │          ▲                  │
│   ┌──────────┐    /imu      │ Node   │──────────┘                  │
│   │   IMU    │─────────────►│        │    /pose                    │
│   │  Driver  │              └────────┘                             │
│   └──────────┘                                                     │
│                                                                     │
│   Legend: ───► Topic (async, many-to-many)                         │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 5.3.2 Nodes

A **node** is the fundamental unit of computation in ROS 2. Each node should perform a single, coherent function. Nodes are:

- **Processes**: Each node runs as a separate process (or thread in a composed system)
- **Named**: Every node has a unique name within its namespace
- **Configurable**: Nodes accept parameters for runtime configuration
- **Discoverable**: Nodes automatically find each other via DDS discovery

**Node Naming Conventions:**
```
/namespace/node_name

Examples:
/robot1/camera_driver
/robot1/lidar_driver
/robot2/camera_driver
/global_planner
```

### 5.3.3 Topics

**Topics** provide asynchronous, many-to-many communication using the publish-subscribe pattern:

```
┌─────────────────────────────────────────────────────────────────────┐
│                     TOPIC COMMUNICATION                             │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Publishers                    Topic                 Subscribers   │
│                                                                     │
│  ┌─────────┐                 ┌─────────┐            ┌─────────┐    │
│  │Publisher│────publish────►│  /scan  │───────────►│Subscriber│   │
│  │   A     │                 │         │            │    X    │    │
│  └─────────┘                 │ Message │            └─────────┘    │
│                              │  Type:  │                           │
│  ┌─────────┐                 │LaserScan│            ┌─────────┐    │
│  │Publisher│────publish────►│         │───────────►│Subscriber│   │
│  │   B     │                 └─────────┘            │    Y    │    │
│  └─────────┘                                        └─────────┘    │
│                                                                     │
│   • Decoupled: Publishers don't know about subscribers              │
│   • Typed: All messages on a topic share the same type              │
│   • Buffered: QoS controls message queuing behavior                 │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**When to Use Topics:**
- Streaming sensor data (continuous, high-frequency)
- State updates (robot pose, joint states)
- Commands that don't require acknowledgment
- One-to-many or many-to-many communication

**Common Topic Types:**

| Topic | Message Type | Typical Frequency |
|-------|--------------|-------------------|
| `/scan` | sensor_msgs/LaserScan | 10-40 Hz |
| `/image_raw` | sensor_msgs/Image | 30-60 Hz |
| `/imu` | sensor_msgs/Imu | 100-400 Hz |
| `/cmd_vel` | geometry_msgs/Twist | 10-50 Hz |
| `/joint_states` | sensor_msgs/JointState | 50-500 Hz |
| `/tf` | tf2_msgs/TFMessage | 100+ Hz |

### 5.3.4 Services

**Services** provide synchronous, request-response communication:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    SERVICE COMMUNICATION                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│      Client                   Service                  Server       │
│                                                                     │
│  ┌─────────┐              ┌───────────────┐        ┌─────────┐     │
│  │         │───Request───►│ /set_position │───────►│         │     │
│  │ Client  │              │               │        │ Server  │     │
│  │         │◄──Response───│  srv type:    │◄───────│         │     │
│  └─────────┘              │SetPosition.srv│        └─────────┘     │
│      │                    └───────────────┘             │          │
│      │                                                  │          │
│      └──────────────── BLOCKING ────────────────────────┘          │
│                                                                     │
│   • Synchronous: Client blocks until response received              │
│   • One-to-one: Single server per service                          │
│   • Typed: Request and response have defined types                  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Service Definition (.srv file):**
```
# SetPosition.srv
# Request
float64 x
float64 y
float64 z
---
# Response
bool success
string message
```

**When to Use Services:**
- Configuration changes (set parameters, change modes)
- Discrete operations (spawn object, save map)
- Queries (get current state, compute transform)
- Operations requiring confirmation

### 5.3.5 Actions

**Actions** provide asynchronous goal-oriented communication with feedback:

```
┌─────────────────────────────────────────────────────────────────────┐
│                     ACTION COMMUNICATION                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│      Client                   Action                   Server       │
│                                                                     │
│  ┌─────────┐    ┌──────────────────────────────────┐   ┌─────────┐ │
│  │         │    │         /navigate_to_pose         │   │         │ │
│  │         │────┼──────── Goal ────────────────────┼──►│         │ │
│  │         │◄───┼──────── Accepted/Rejected ───────┼───│         │ │
│  │  Action │    │                                  │   │  Action │ │
│  │  Client │◄───┼──────── Feedback (periodic) ─────┼───│  Server │ │
│  │         │    │                                  │   │         │ │
│  │         │◄───┼──────── Result ──────────────────┼───│         │ │
│  │         │────┼──────── Cancel (optional) ───────┼──►│         │ │
│  └─────────┘    └──────────────────────────────────┘   └─────────┘ │
│                                                                     │
│   • Asynchronous: Client doesn't block                              │
│   • Preemptable: Goals can be canceled                             │
│   • Feedback: Progress updates during execution                     │
│   • Stateful: Track goal lifecycle                                  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Action Definition (.action file):**
```
# NavigateToPose.action
# Goal
geometry_msgs/PoseStamped pose
---
# Result
bool success
float64 time_elapsed
---
# Feedback
geometry_msgs/PoseStamped current_pose
float64 distance_remaining
float64 estimated_time_remaining
```

**When to Use Actions:**
- Long-running tasks (navigation, manipulation)
- Tasks requiring progress feedback
- Preemptable operations
- Tasks with distinct success/failure outcomes

### 5.3.6 Communication Pattern Selection

```
┌─────────────────────────────────────────────────────────────────────┐
│              CHOOSING THE RIGHT COMMUNICATION PATTERN               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  Is it streaming/continuous data?                                   │
│       │                                                             │
│       ├── YES ──► TOPIC                                            │
│       │           (sensor data, state updates, commands)           │
│       │                                                             │
│       └── NO ──► Does it need progress feedback?                   │
│                       │                                             │
│                       ├── YES ──► ACTION                           │
│                       │           (navigation, manipulation)        │
│                       │                                             │
│                       └── NO ──► Is it a quick operation?          │
│                                       │                             │
│                                       ├── YES ──► SERVICE          │
│                                       │           (config, queries) │
│                                       │                             │
│                                       └── NO ──► ACTION            │
│                                                   (long operations) │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 5.4 Middleware and DDS: Quality of Service

### 5.4.1 Data Distribution Service (DDS)

ROS 2 uses the **Data Distribution Service (DDS)** standard for communication, replacing the custom TCPROS/UDPROS protocols of ROS 1. DDS is an industry-standard middleware used in aerospace, defense, and financial systems.

**DDS Benefits for Robotics:**
- **Decentralized discovery**: No single point of failure
- **Configurable reliability**: Match communication to requirements
- **Security**: Built-in authentication and encryption
- **Real-time support**: Deterministic delivery guarantees
- **Multi-platform**: Interoperability across vendors

**Available DDS Implementations:**

| Implementation | Vendor | License | Default In |
|---------------|--------|---------|------------|
| Fast-DDS | eProsima | Apache 2.0 | Humble, Jazzy |
| Cyclone DDS | Eclipse | EPL 2.0 | Alternative |
| RTI Connext | RTI | Commercial | Commercial use |

### 5.4.2 Quality of Service (QoS) Profiles

QoS policies control how messages are delivered between publishers and subscribers:

```
┌─────────────────────────────────────────────────────────────────────┐
│                     QoS POLICY DIMENSIONS                           │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  RELIABILITY                                                        │
│  ├── BEST_EFFORT: May lose messages (low latency)                  │
│  └── RELIABLE: Guaranteed delivery (higher latency)                │
│                                                                     │
│  DURABILITY                                                         │
│  ├── VOLATILE: No persistence (new subscribers miss old messages)  │
│  ├── TRANSIENT_LOCAL: Keep last message for late subscribers       │
│  └── TRANSIENT/PERSISTENT: Survive node restarts                   │
│                                                                     │
│  HISTORY                                                            │
│  ├── KEEP_LAST(n): Keep only last n messages                       │
│  └── KEEP_ALL: Keep all messages (memory risk)                     │
│                                                                     │
│  LIVELINESS                                                         │
│  ├── AUTOMATIC: System determines liveness                         │
│  └── MANUAL: Application must assert liveness                      │
│                                                                     │
│  DEADLINE                                                           │
│  └── Expected message interval (triggers callback if missed)       │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 5.4.3 Predefined QoS Profiles

ROS 2 provides preset QoS profiles for common use cases:

```python
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    qos_profile_sensor_data,    # Best-effort, volatile, depth=5
    qos_profile_system_default, # Reliable, volatile, depth=10
    qos_profile_services_default,
    qos_profile_parameters,
)
```

**Profile Characteristics:**

| Profile | Reliability | Durability | History Depth | Use Case |
|---------|-------------|------------|---------------|----------|
| sensor_data | Best Effort | Volatile | 5 | LiDAR, cameras |
| system_default | Reliable | Volatile | 10 | General purpose |
| services | Reliable | Volatile | 10 | Service calls |
| parameters | Reliable | Volatile | 1000 | Parameter updates |

### 5.4.4 Custom QoS Configuration

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Custom profile for critical control commands
control_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,  # Only need latest command
    deadline=Duration(seconds=0, nanoseconds=50_000_000),  # 50ms deadline
)

# Custom profile for high-frequency sensor data
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,  # Only latest reading matters
)
```

### 5.4.5 QoS Compatibility

Publishers and subscribers must have compatible QoS settings to communicate:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    QoS COMPATIBILITY RULES                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  Publisher                    Subscriber           Compatible?      │
│  ──────────────────────────────────────────────────────────────     │
│  RELIABLE          ◄─────────► RELIABLE            ✓ Yes            │
│  RELIABLE          ◄─────────► BEST_EFFORT         ✓ Yes            │
│  BEST_EFFORT       ◄─────────► BEST_EFFORT         ✓ Yes            │
│  BEST_EFFORT       ◄─────────► RELIABLE            ✗ No!            │
│                                                                     │
│  TRANSIENT_LOCAL   ◄─────────► TRANSIENT_LOCAL     ✓ Yes            │
│  TRANSIENT_LOCAL   ◄─────────► VOLATILE            ✓ Yes            │
│  VOLATILE          ◄─────────► VOLATILE            ✓ Yes            │
│  VOLATILE          ◄─────────► TRANSIENT_LOCAL     ✗ No!            │
│                                                                     │
│  Rule: Publisher QoS must be >= Subscriber requirements             │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 5.5 The rclpy Client Library

### 5.5.1 ROS 2 Client Libraries

ROS 2 provides client libraries for multiple languages:

- **rclcpp**: C++ client library (performance-critical applications)
- **rclpy**: Python client library (prototyping, high-level logic)
- **rclrs**: Rust client library (safety-critical applications)
- **rclnodejs**: Node.js client library (web integration)

All client libraries share a common underlying C library (rcl), ensuring consistent behavior across languages.

### 5.5.2 Basic rclpy Node Structure

```python
#!/usr/bin/env python3
"""
minimal_node.py - The simplest possible ROS 2 node
"""

import rclpy
from rclpy.node import Node


class MinimalNode(Node):
    """A minimal ROS 2 node that logs a message once."""

    def __init__(self):
        # Initialize the node with a unique name
        super().__init__('minimal_node')

        # Log a message at INFO level
        self.get_logger().info('Minimal node has started!')


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of our node
    node = MinimalNode()

    try:
        # Keep the node running until shutdown
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 5.5.3 Node Lifecycle

```
┌─────────────────────────────────────────────────────────────────────┐
│                    NODE EXECUTION LIFECYCLE                         │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│     ┌─────────────────────────────────────────────────────────┐    │
│     │                    rclpy.init()                          │    │
│     │              Initialize ROS context                      │    │
│     └─────────────────────────┬───────────────────────────────┘    │
│                               ▼                                     │
│     ┌─────────────────────────────────────────────────────────┐    │
│     │                  Node.__init__()                         │    │
│     │           Create node, publishers, subscribers           │    │
│     │                 timers, services, etc.                   │    │
│     └─────────────────────────┬───────────────────────────────┘    │
│                               ▼                                     │
│     ┌─────────────────────────────────────────────────────────┐    │
│     │                    rclpy.spin()                          │    │
│     │         ┌─────────────────────────────────┐             │    │
│     │         │     Process callbacks:          │             │    │
│     │         │     • Timer callbacks           │◄────────┐   │    │
│     │         │     • Subscription callbacks    │         │   │    │
│     │         │     • Service callbacks         │─────────┘   │    │
│     │         │     • Action callbacks          │   loop      │    │
│     │         └─────────────────────────────────┘             │    │
│     └─────────────────────────┬───────────────────────────────┘    │
│                               ▼ (Ctrl+C or shutdown)               │
│     ┌─────────────────────────────────────────────────────────┐    │
│     │                node.destroy_node()                       │    │
│     │               rclpy.shutdown()                           │    │
│     │              Clean up resources                          │    │
│     └─────────────────────────────────────────────────────────┘    │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 5.5.4 Timers

Timers execute callbacks at regular intervals:

```python
class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')

        # Create a timer that fires every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        """Called every 0.5 seconds."""
        self.counter += 1
        self.get_logger().info(f'Timer callback #{self.counter}')
```

### 5.5.5 Logging

ROS 2 provides structured logging with severity levels:

```python
# Logging levels (in order of severity)
self.get_logger().debug('Detailed debugging information')
self.get_logger().info('General informational messages')
self.get_logger().warn('Warning about potential issues')
self.get_logger().error('Error conditions')
self.get_logger().fatal('Critical errors, system may crash')

# Throttled logging (once per interval)
self.get_logger().info('Sensor reading', throttle_duration_sec=1.0)

# Conditional logging
self.get_logger().info('Condition met', once=True)
```

---

## 5.6 Packages and Workspace Organization

### 5.6.1 Workspace Structure

A ROS 2 workspace is a directory containing packages:

```
my_robot_ws/                     # Workspace root
├── src/                         # Source space (your packages)
│   ├── my_robot_description/    # URDF/meshes package
│   ├── my_robot_bringup/        # Launch files package
│   ├── my_robot_control/        # Control nodes package
│   └── my_robot_navigation/     # Navigation package
├── build/                       # Build space (generated)
├── install/                     # Install space (generated)
└── log/                         # Log space (generated)
```

### 5.6.2 Package Structure (Python)

```
my_package/
├── package.xml                  # Package metadata and dependencies
├── setup.py                     # Python build configuration
├── setup.cfg                    # Additional build settings
├── resource/
│   └── my_package               # Package marker file
├── my_package/                  # Python source directory
│   ├── __init__.py
│   ├── node_one.py             # Node implementation
│   └── node_two.py             # Another node
├── launch/                      # Launch files
│   └── robot.launch.py
├── config/                      # Configuration files
│   └── params.yaml
└── test/                        # Unit tests
    └── test_node_one.py
```

### 5.6.3 package.xml

The package manifest declares metadata and dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_control</name>
  <version>0.1.0</version>
  <description>Control nodes for my robot</description>
  <maintainer email="developer@example.com">Developer Name</maintainer>
  <license>Apache-2.0</license>

  <!-- Build tool -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Runtime dependencies -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>

  <!-- Test dependencies -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 5.6.4 setup.py

The Python setup configuration:

```python
from setuptools import setup

package_name = 'my_robot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        ('share/' + package_name + '/launch',
            ['launch/robot.launch.py']),
        # Install config files
        ('share/' + package_name + '/config',
            ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer Name',
    maintainer_email='developer@example.com',
    description='Control nodes for my robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_controller = my_robot_control.velocity_controller:main',
            'state_estimator = my_robot_control.state_estimator:main',
        ],
    },
)
```

### 5.6.5 Building with colcon

**colcon** is the build tool for ROS 2:

```bash
# Navigate to workspace root
cd ~/my_robot_ws

# Build all packages
colcon build

# Build specific package
colcon build --packages-select my_robot_control

# Build with symlinks (for Python development)
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Verify package is available
ros2 pkg list | grep my_robot
```

### 5.6.6 Creating a New Package

```bash
# Create a Python package
cd ~/my_robot_ws/src
ros2 pkg create --build-type ament_python my_new_package \
    --dependencies rclpy std_msgs

# Create a C++ package
ros2 pkg create --build-type ament_cmake my_cpp_package \
    --dependencies rclcpp std_msgs
```

---

## 5.7 Publisher/Subscriber Implementation

### 5.7.1 Complete Publisher Node

```python
#!/usr/bin/env python3
"""
velocity_publisher.py - Publishes velocity commands
Demonstrates: Publisher, Timer, QoS configuration
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
import math


class VelocityPublisher(Node):
    """
    Publishes sinusoidal velocity commands for robot motion.

    Parameters:
        linear_amplitude (float): Maximum linear velocity (m/s)
        angular_amplitude (float): Maximum angular velocity (rad/s)
        frequency (float): Oscillation frequency (Hz)
        publish_rate (float): Message publish rate (Hz)
    """

    def __init__(self):
        super().__init__('velocity_publisher')

        # Declare parameters with default values
        self.declare_parameter('linear_amplitude', 0.5)
        self.declare_parameter('angular_amplitude', 1.0)
        self.declare_parameter('frequency', 0.1)
        self.declare_parameter('publish_rate', 10.0)

        # Get parameter values
        self.linear_amp = self.get_parameter('linear_amplitude').value
        self.angular_amp = self.get_parameter('angular_amplitude').value
        self.freq = self.get_parameter('frequency').value
        publish_rate = self.get_parameter('publish_rate').value

        # Configure QoS for reliable command delivery
        cmd_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Create publisher
        self.publisher = self.create_publisher(
            Twist,           # Message type
            'cmd_vel',       # Topic name
            cmd_qos          # QoS profile
        )

        # Create timer for periodic publishing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_velocity)

        # Track time for sinusoidal generation
        self.start_time = self.get_clock().now()

        self.get_logger().info(
            f'Velocity publisher started: '
            f'linear_amp={self.linear_amp}, angular_amp={self.angular_amp}'
        )

    def publish_velocity(self):
        """Generate and publish velocity command."""
        # Calculate elapsed time
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # Generate sinusoidal velocities
        msg = Twist()
        msg.linear.x = self.linear_amp * math.sin(2 * math.pi * self.freq * elapsed)
        msg.angular.z = self.angular_amp * math.cos(2 * math.pi * self.freq * elapsed)

        # Publish the message
        self.publisher.publish(msg)

        # Log periodically (throttled)
        self.get_logger().debug(
            f'Publishing: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down velocity publisher')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 5.7.2 Complete Subscriber Node

```python
#!/usr/bin/env python3
"""
velocity_subscriber.py - Subscribes to velocity commands
Demonstrates: Subscriber, callback handling, message processing
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from collections import deque
import statistics


class VelocitySubscriber(Node):
    """
    Subscribes to velocity commands and computes statistics.

    Tracks recent velocities and computes moving averages
    for monitoring robot motion.
    """

    def __init__(self):
        super().__init__('velocity_subscriber')

        # Declare parameters
        self.declare_parameter('window_size', 10)
        self.window_size = self.get_parameter('window_size').value

        # Rolling windows for statistics
        self.linear_history = deque(maxlen=self.window_size)
        self.angular_history = deque(maxlen=self.window_size)

        # Create subscription
        self.subscription = self.create_subscription(
            Twist,                   # Message type
            'cmd_vel',               # Topic name
            self.velocity_callback,  # Callback function
            10                       # QoS depth
        )

        # Statistics timer
        self.stats_timer = self.create_timer(2.0, self.log_statistics)

        # Message counter
        self.msg_count = 0

        self.get_logger().info('Velocity subscriber started')

    def velocity_callback(self, msg: Twist):
        """Process incoming velocity command."""
        self.msg_count += 1

        # Store values in rolling windows
        self.linear_history.append(msg.linear.x)
        self.angular_history.append(msg.angular.z)

        # Log each message at debug level
        self.get_logger().debug(
            f'Received cmd #{self.msg_count}: '
            f'linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}'
        )

    def log_statistics(self):
        """Periodically log velocity statistics."""
        if len(self.linear_history) < 2:
            self.get_logger().info('Waiting for messages...')
            return

        linear_mean = statistics.mean(self.linear_history)
        linear_std = statistics.stdev(self.linear_history)
        angular_mean = statistics.mean(self.angular_history)
        angular_std = statistics.stdev(self.angular_history)

        self.get_logger().info(
            f'Stats over {len(self.linear_history)} samples: '
            f'linear={linear_mean:.3f}±{linear_std:.3f} m/s, '
            f'angular={angular_mean:.3f}±{angular_std:.3f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = VelocitySubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down velocity subscriber')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 5.7.3 Combined Publisher/Subscriber Node

```python
#!/usr/bin/env python3
"""
velocity_processor.py - Subscribes, processes, and republishes
Demonstrates: Combined pub/sub, message transformation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class VelocityProcessor(Node):
    """
    Subscribes to velocity commands, applies limits, and republishes.
    Also publishes derived metrics (speed, turn rate).
    """

    def __init__(self):
        super().__init__('velocity_processor')

        # Velocity limits
        self.declare_parameter('max_linear', 1.0)
        self.declare_parameter('max_angular', 2.0)
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value

        # Subscriber
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel_raw', self.process_velocity, 10
        )

        # Publishers
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed_pub = self.create_publisher(Float64, 'speed', 10)

        self.get_logger().info(
            f'Velocity processor: max_linear={self.max_linear}, '
            f'max_angular={self.max_angular}'
        )

    def process_velocity(self, msg: Twist):
        """Apply limits and republish."""
        # Clamp velocities to limits
        limited = Twist()
        limited.linear.x = max(-self.max_linear,
                               min(self.max_linear, msg.linear.x))
        limited.linear.y = max(-self.max_linear,
                               min(self.max_linear, msg.linear.y))
        limited.angular.z = max(-self.max_angular,
                                min(self.max_angular, msg.angular.z))

        # Publish limited velocity
        self.vel_pub.publish(limited)

        # Compute and publish speed magnitude
        speed = Float64()
        speed.data = (limited.linear.x**2 + limited.linear.y**2)**0.5
        self.speed_pub.publish(speed)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityProcessor()
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

---

## 5.8 Launch Files

### 5.8.1 Purpose of Launch Files

Launch files orchestrate the startup of multiple nodes with configured parameters:

- Start multiple nodes from a single command
- Pass parameters and arguments to nodes
- Set namespace and remappings
- Compose subsystems hierarchically
- Manage node lifecycle

### 5.8.2 Python Launch File Structure

```python
#!/usr/bin/env python3
"""
robot.launch.py - Launch the complete robot system
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the launch description."""

    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='my_robot',
        description='Name of the robot'
    )

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation time'
    )

    # Get package share directory
    pkg_share = FindPackageShare('my_robot_bringup')

    # Define nodes
    velocity_controller = Node(
        package='my_robot_control',
        executable='velocity_controller',
        name='velocity_controller',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[{
            'max_linear_velocity': 1.0,
            'max_angular_velocity': 2.0,
            'use_sim_time': LaunchConfiguration('use_sim'),
        }],
        remappings=[
            ('cmd_vel_raw', 'teleop/cmd_vel'),
            ('cmd_vel', 'base/cmd_vel'),
        ],
        output='screen',
    )

    state_estimator = Node(
        package='my_robot_control',
        executable='state_estimator',
        name='state_estimator',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'estimator.yaml'])
        ],
        output='screen',
    )

    # Return the launch description
    return LaunchDescription([
        robot_name_arg,
        use_sim_arg,
        velocity_controller,
        state_estimator,
    ])
```

### 5.8.3 Launch Actions

```python
from launch.actions import (
    DeclareLaunchArgument,      # Declare configurable arguments
    IncludeLaunchDescription,   # Include another launch file
    GroupAction,                # Group actions together
    TimerAction,                # Delay action execution
    ExecuteProcess,             # Run arbitrary process
    SetEnvironmentVariable,     # Set environment variables
    LogInfo,                    # Log messages
)
from launch.conditions import (
    IfCondition,                # Conditional execution
    UnlessCondition,            # Inverse conditional
)
```

### 5.8.4 Including Other Launch Files

```python
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Include launch file from another package
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': 'my_world.sdf',
            'verbose': 'true',
        }.items()
    )

    return LaunchDescription([gazebo_launch])
```

### 5.8.5 Conditional Launch

```python
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', 'config/robot.rviz'],
    )

    return LaunchDescription([use_rviz_arg, rviz_node])
```

---

## 5.9 Parameters and Dynamic Configuration

### 5.9.1 The Parameter System

Parameters provide runtime configuration for nodes:

```
┌─────────────────────────────────────────────────────────────────────┐
│                      PARAMETER SYSTEM                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌───────────────────────────────────────────────────────────┐    │
│   │                      NODE                                  │    │
│   │  ┌─────────────────────────────────────────────────────┐  │    │
│   │  │             PARAMETER SERVER                         │  │    │
│   │  │                                                      │  │    │
│   │  │  max_velocity: 1.0                                   │  │    │
│   │  │  control_frequency: 50.0                             │  │    │
│   │  │  pid.kp: 1.5                                         │  │    │
│   │  │  pid.ki: 0.1                                         │  │    │
│   │  │  pid.kd: 0.05                                        │  │    │
│   │  │                                                      │  │    │
│   │  └────────────┬─────────────────────────────────────────┘  │    │
│   │               │                                            │    │
│   │               ▼                                            │    │
│   │  ┌─────────────────────────────────────────────────────┐  │    │
│   │  │         PARAMETER SERVICES (auto-created)           │  │    │
│   │  │  ~/get_parameters                                   │  │    │
│   │  │  ~/set_parameters                                   │  │    │
│   │  │  ~/list_parameters                                  │  │    │
│   │  │  ~/describe_parameters                              │  │    │
│   │  └─────────────────────────────────────────────────────┘  │    │
│   └───────────────────────────────────────────────────────────┘    │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 5.9.2 Declaring Parameters

```python
class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')

        # Declare parameters with default values
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('enabled', True)
        self.declare_parameter('mode', 'auto')

        # Declare with descriptor for better documentation
        from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange

        self.declare_parameter(
            'pid.kp',
            1.0,
            ParameterDescriptor(
                description='Proportional gain',
                floating_point_range=[FloatingPointRange(
                    from_value=0.0,
                    to_value=10.0,
                    step=0.1
                )]
            )
        )
```

### 5.9.3 Reading and Using Parameters

```python
class ParameterizedController(Node):
    def __init__(self):
        super().__init__('parameterized_controller')

        # Declare parameters
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.05)
        self.declare_parameter('rate', 50.0)

        # Read parameter values
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        rate = self.get_parameter('rate').value

        # Use parameters
        self.timer = self.create_timer(1.0/rate, self.control_loop)

        self.get_logger().info(
            f'Controller initialized: kp={self.kp}, ki={self.ki}, kd={self.kd}'
        )
```

### 5.9.4 Parameter Callbacks

```python
from rcl_interfaces.msg import SetParametersResult

class DynamicController(Node):
    def __init__(self):
        super().__init__('dynamic_controller')

        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.1)

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value

        # Register callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """Handle dynamic parameter updates."""
        for param in params:
            if param.name == 'kp':
                if param.value < 0:
                    return SetParametersResult(
                        successful=False,
                        reason='kp must be non-negative'
                    )
                self.kp = param.value
                self.get_logger().info(f'Updated kp to {self.kp}')

            elif param.name == 'ki':
                if param.value < 0:
                    return SetParametersResult(
                        successful=False,
                        reason='ki must be non-negative'
                    )
                self.ki = param.value
                self.get_logger().info(f'Updated ki to {self.ki}')

        return SetParametersResult(successful=True)
```

### 5.9.5 Parameter YAML Files

```yaml
# config/controller_params.yaml
velocity_controller:
  ros__parameters:
    max_linear_velocity: 1.0
    max_angular_velocity: 2.0
    control_frequency: 50.0

    pid:
      linear:
        kp: 1.5
        ki: 0.1
        kd: 0.05
      angular:
        kp: 2.0
        ki: 0.2
        kd: 0.1

    enabled_features:
      - velocity_limiting
      - acceleration_limiting
      - collision_avoidance
```

### 5.9.6 CLI Parameter Commands

```bash
# List all parameters for a node
ros2 param list /velocity_controller

# Get a parameter value
ros2 param get /velocity_controller kp

# Set a parameter value
ros2 param set /velocity_controller kp 2.0

# Dump all parameters to file
ros2 param dump /velocity_controller

# Load parameters from file
ros2 param load /velocity_controller params.yaml
```

---

## 5.10 URDF: Robot Description

### 5.10.1 What is URDF?

The **Unified Robot Description Format (URDF)** is an XML format for describing robot structure:

```
┌─────────────────────────────────────────────────────────────────────┐
│                        URDF COMPONENTS                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   URDF describes a robot as a tree of:                              │
│                                                                     │
│   ┌─────────────┐         ┌─────────────┐         ┌─────────────┐  │
│   │   LINKS     │◄───────►│   JOINTS    │◄───────►│   LINKS     │  │
│   │  (bodies)   │         │ (connections)│        │  (bodies)   │  │
│   └─────────────┘         └─────────────┘         └─────────────┘  │
│                                                                     │
│   Links have:                    Joints have:                       │
│   • Visual geometry              • Type (revolute, prismatic, etc.)│
│   • Collision geometry           • Parent and child links          │
│   • Inertial properties          • Axis of rotation/translation    │
│   • Material/color               • Limits (position, velocity)     │
│                                                                     │
│                   ┌─────────────────────────────┐                  │
│                   │      ROBOT TREE             │                  │
│                   │                             │                  │
│                   │      base_link              │                  │
│                   │          │                  │                  │
│                   │    ┌─────┴─────┐            │                  │
│                   │    ▼           ▼            │                  │
│                   │  wheel_L    wheel_R         │                  │
│                   │    │           │            │                  │
│                   │    ▼           ▼            │                  │
│                   │  caster_L   caster_R        │                  │
│                   │                             │                  │
│                   └─────────────────────────────┘                  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 5.10.2 Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_robot">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.04" ixy="0" ixz="0"
               iyy="0.1" iyz="0" izz="0.12"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Joint connecting base to left wheel -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.175 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

</robot>
```

### 5.10.3 Joint Types

| Type | Motion | Example |
|------|--------|---------|
| `fixed` | None | Sensor mount |
| `continuous` | Unlimited rotation | Wheel |
| `revolute` | Limited rotation | Arm joint |
| `prismatic` | Linear translation | Linear actuator |
| `floating` | 6-DOF | Free-flying base |
| `planar` | 2D plane motion | XY table |

### 5.10.4 xacro: URDF Macros

**xacro** extends URDF with macros and parameters:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot">

  <!-- Properties (variables) -->
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>
  <xacro:property name="base_length" value="0.5"/>

  <!-- Macro definition -->
  <xacro:macro name="wheel" params="prefix y_offset">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="0 ${y_offset} ${-wheel_radius}" rpy="1.5708 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:wheel prefix="left" y_offset="0.175"/>
  <xacro:wheel prefix="right" y_offset="-0.175"/>

</robot>
```

### 5.10.5 Publishing Robot State

```python
#!/usr/bin/env python3
"""
robot_state_publisher_example.py
Demonstrates using robot_state_publisher with joint states
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class JointStatePublisher(Node):
    """Publishes joint states for visualization."""

    def __init__(self):
        super().__init__('joint_state_publisher')

        self.publisher = self.create_publisher(
            JointState, 'joint_states', 10
        )

        self.timer = self.create_timer(0.02, self.publish_joint_states)
        self.t = 0.0

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']

        # Simulate wheel rotation
        wheel_vel = 1.0  # rad/s
        self.t += 0.02
        position = wheel_vel * self.t

        msg.position = [position, position]
        msg.velocity = [wheel_vel, wheel_vel]

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## 5.11 ros2_control Framework Overview

### 5.11.1 Architecture

The **ros2_control** framework provides a standardized interface between high-level controllers and hardware:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    ros2_control ARCHITECTURE                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌─────────────────────────────────────────────────────────────┐  │
│   │                    CONTROLLER MANAGER                        │  │
│   │         (Loads, configures, starts controllers)              │  │
│   └──────────────────────────┬──────────────────────────────────┘  │
│                              │                                      │
│         ┌────────────────────┼────────────────────┐                │
│         ▼                    ▼                    ▼                │
│   ┌───────────┐        ┌───────────┐        ┌───────────┐         │
│   │Diff Drive │        │  Joint    │        │  Gripper  │         │
│   │Controller │        │Trajectory │        │Controller │         │
│   └─────┬─────┘        └─────┬─────┘        └─────┬─────┘         │
│         │                    │                    │                │
│   ┌─────┴────────────────────┴────────────────────┴─────┐         │
│   │              RESOURCE MANAGER                        │         │
│   │     (Manages state/command interfaces)               │         │
│   └─────────────────────────┬───────────────────────────┘         │
│                             │                                      │
│   ┌─────────────────────────┴───────────────────────────┐         │
│   │              HARDWARE INTERFACE                      │         │
│   │     (Abstracts real/simulated hardware)              │         │
│   └─────────────────────────┬───────────────────────────┘         │
│                             │                                      │
│         ┌───────────────────┼───────────────────┐                 │
│         ▼                   ▼                   ▼                 │
│   ┌───────────┐       ┌───────────┐       ┌───────────┐          │
│   │  Motor 1  │       │  Motor 2  │       │  Gripper  │          │
│   └───────────┘       └───────────┘       └───────────┘          │
│                                                                    │
└────────────────────────────────────────────────────────────────────┘
```

### 5.11.2 URDF ros2_control Tags

```xml
<ros2_control name="my_robot_system" type="system">
  <hardware>
    <plugin>my_robot_hardware/MyRobotHardware</plugin>
    <param name="serial_port">/dev/ttyUSB0</param>
  </hardware>

  <joint name="left_wheel_joint">
    <command_interface name="velocity">
      <param name="min">-10.0</param>
      <param name="max">10.0</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="right_wheel_joint">
    <command_interface name="velocity">
      <param name="min">-10.0</param>
      <param name="max">10.0</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

### 5.11.3 Controller Configuration

```yaml
# config/controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.35
    wheel_radius: 0.1

    publish_rate: 50.0
    odom_frame_id: odom
    base_frame_id: base_link

    linear.x.max_velocity: 1.0
    angular.z.max_velocity: 2.0
```

---

## 5.12 Lifecycle Nodes

### 5.12.1 Managed Node Lifecycle

ROS 2 provides **lifecycle nodes** for deterministic startup and shutdown:

```
┌─────────────────────────────────────────────────────────────────────┐
│                   LIFECYCLE NODE STATES                             │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│    ┌──────────────┐                                                │
│    │  Unconfigured │◄────────────── create()                       │
│    └───────┬───────┘                                               │
│            │ configure()                                            │
│            ▼                                                        │
│    ┌──────────────┐                                                │
│    │   Inactive   │◄────────────── on_configure()                  │
│    └───────┬───────┘                                               │
│            │ activate()                                             │
│            ▼                                                        │
│    ┌──────────────┐                                                │
│    │    Active    │◄────────────── on_activate()                   │
│    └───────┬───────┘                                               │
│            │ deactivate()                                           │
│            ▼                                                        │
│    ┌──────────────┐                                                │
│    │   Inactive   │                                                │
│    └───────┬───────┘                                               │
│            │ cleanup()                                              │
│            ▼                                                        │
│    ┌──────────────┐                                                │
│    │ Unconfigured │                                                │
│    └───────┬───────┘                                               │
│            │ shutdown()                                             │
│            ▼                                                        │
│    ┌──────────────┐                                                │
│    │   Finalized  │                                                │
│    └──────────────┘                                                │
│                                                                     │
│   Error Handling:                                                   │
│   • Any state can transition to ErrorProcessing on failure          │
│   • From error, can cleanup() → Unconfigured or shutdown()          │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 5.12.2 Implementing a Lifecycle Node

```python
#!/usr/bin/env python3
"""
lifecycle_sensor.py - Lifecycle-managed sensor node
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import State
from sensor_msgs.msg import LaserScan


class LifecycleSensor(LifecycleNode):
    """A sensor node with managed lifecycle."""

    def __init__(self):
        super().__init__('lifecycle_sensor')
        self.publisher_ = None
        self.timer_ = None
        self.get_logger().info('Lifecycle sensor created (unconfigured)')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure the node (allocate resources)."""
        self.get_logger().info('Configuring...')

        self.declare_parameter('scan_frequency', 10.0)
        self.scan_frequency = self.get_parameter('scan_frequency').value

        # Create publisher (but don't start publishing yet)
        self.publisher_ = self.create_lifecycle_publisher(
            LaserScan, 'scan', 10
        )

        self.get_logger().info('Configured successfully')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate the node (start publishing)."""
        self.get_logger().info('Activating...')

        # Start the publishing timer
        self.timer_ = self.create_timer(
            1.0 / self.scan_frequency,
            self.publish_scan
        )

        self.get_logger().info('Activated - publishing sensor data')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate the node (stop publishing)."""
        self.get_logger().info('Deactivating...')

        # Stop the timer
        if self.timer_:
            self.timer_.cancel()
            self.timer_ = None

        self.get_logger().info('Deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Clean up resources."""
        self.get_logger().info('Cleaning up...')

        # Destroy publisher
        if self.publisher_:
            self.destroy_publisher(self.publisher_)
            self.publisher_ = None

        self.get_logger().info('Cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Handle shutdown."""
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS

    def publish_scan(self):
        """Publish scan data (only when active)."""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        # ... fill scan data ...
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### 5.12.3 Lifecycle CLI Commands

```bash
# List lifecycle nodes
ros2 lifecycle nodes

# Get current state
ros2 lifecycle get /lifecycle_sensor

# Trigger transitions
ros2 lifecycle set /lifecycle_sensor configure
ros2 lifecycle set /lifecycle_sensor activate
ros2 lifecycle set /lifecycle_sensor deactivate
ros2 lifecycle set /lifecycle_sensor cleanup
ros2 lifecycle set /lifecycle_sensor shutdown
```

---

## 5.13 TF2: Coordinate Transforms

### 5.13.1 The Transform Tree

**TF2** maintains a tree of coordinate frames and their relationships:

```
┌─────────────────────────────────────────────────────────────────────┐
│                     TF2 TRANSFORM TREE                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│                        map                                          │
│                         │                                           │
│                    (localization)                                   │
│                         │                                           │
│                         ▼                                           │
│                        odom                                         │
│                         │                                           │
│                    (odometry)                                       │
│                         │                                           │
│                         ▼                                           │
│                     base_link                                       │
│               ┌─────────┼─────────┐                                │
│               │         │         │                                 │
│               ▼         ▼         ▼                                 │
│          laser_link  camera_link  imu_link                         │
│               │                                                     │
│               ▼                                                     │
│          laser_frame                                                │
│                                                                     │
│   Key Concepts:                                                     │
│   • Each arrow is a transform (translation + rotation)              │
│   • Transforms are timestamped                                      │
│   • Tree structure: single parent per frame                         │
│   • Can query any frame relative to any other                       │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 5.13.2 Standard Frame Conventions (REP 105)

| Frame | Description | Published By |
|-------|-------------|--------------|
| `map` | Global fixed frame (world) | SLAM/localization |
| `odom` | Odometry frame (smooth, drifts) | Odometry |
| `base_link` | Robot body frame | Static (robot center) |
| `base_footprint` | Ground projection of base | Static |
| `*_link` | Sensor/component frames | Static (URDF) |

### 5.13.3 Broadcasting Transforms

```python
#!/usr/bin/env python3
"""
tf_broadcaster.py - Broadcasts transforms to TF2
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math


class OdometryBroadcaster(Node):
    """Broadcasts odometry transform (odom → base_link)."""

    def __init__(self):
        super().__init__('odometry_broadcaster')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.02, self.broadcast_transform)

        # Simulated robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.1  # m/s
        self.vtheta = 0.05  # rad/s

    def broadcast_transform(self):
        """Broadcast current odometry transform."""
        # Update position (simple integration)
        dt = 0.02
        self.x += self.vx * math.cos(self.theta) * dt
        self.y += self.vx * math.sin(self.theta) * dt
        self.theta += self.vtheta * dt

        # Create transform message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Rotation (quaternion from yaw)
        t.transform.rotation.z = math.sin(self.theta / 2)
        t.transform.rotation.w = math.cos(self.theta / 2)

        # Broadcast
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### 5.13.4 Static Transforms

For fixed transforms (sensor mounts), use static broadcaster:

```python
from tf2_ros import StaticTransformBroadcaster

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_frame_publisher')

        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transform once
        self.publish_static_transforms()

    def publish_static_transforms(self):
        # Laser mounted 20cm forward, 10cm up
        laser_tf = TransformStamped()
        laser_tf.header.stamp = self.get_clock().now().to_msg()
        laser_tf.header.frame_id = 'base_link'
        laser_tf.child_frame_id = 'laser_link'
        laser_tf.transform.translation.x = 0.20
        laser_tf.transform.translation.z = 0.10
        laser_tf.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform(laser_tf)
```

### 5.13.5 Listening to Transforms

```python
#!/usr/bin/env python3
"""
tf_listener.py - Listens to TF2 transforms
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class TransformListenerNode(Node):
    """Listens to transforms and converts points between frames."""

    def __init__(self):
        super().__init__('transform_listener')

        # TF2 buffer stores transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically look up transforms
        self.timer = self.create_timer(1.0, self.lookup_transform)

    def lookup_transform(self):
        """Look up transform from laser to map."""
        try:
            # Get transform at current time
            transform = self.tf_buffer.lookup_transform(
                'map',           # Target frame
                'laser_link',    # Source frame
                rclpy.time.Time()  # Time (0 = latest)
            )

            self.get_logger().info(
                f'Laser in map: x={transform.transform.translation.x:.2f}, '
                f'y={transform.transform.translation.y:.2f}'
            )

        except (LookupException, ConnectivityException,
                ExtrapolationException) as e:
            self.get_logger().warn(f'Transform lookup failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TransformListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### 5.13.6 TF2 CLI Commands

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Echo transform
ros2 run tf2_ros tf2_echo map base_link

# Monitor TF
ros2 run tf2_ros tf2_monitor

# Static transform publisher (command line)
ros2 run tf2_ros static_transform_publisher \
    0.2 0 0.1 0 0 0 base_link laser_link
```

---

## 5.14 Essential ROS 2 Tools

### 5.14.1 ros2 CLI Commands

```bash
# Node information
ros2 node list                    # List running nodes
ros2 node info /node_name         # Node details

# Topic information
ros2 topic list                   # List topics
ros2 topic info /topic_name       # Topic details
ros2 topic echo /topic_name       # Print messages
ros2 topic hz /topic_name         # Message frequency
ros2 topic pub /topic_name TYPE 'data'  # Publish message

# Service information
ros2 service list                 # List services
ros2 service call /srv TYPE 'request'   # Call service

# Action information
ros2 action list                  # List actions
ros2 action info /action_name     # Action details
ros2 action send_goal /action_name TYPE 'goal'  # Send goal

# Interface (message/service/action) info
ros2 interface list               # List all interfaces
ros2 interface show TYPE          # Show definition
ros2 interface proto TYPE         # Show prototype

# Run nodes
ros2 run package_name executable  # Run a node
ros2 launch package_name file.launch.py  # Run launch file
```

### 5.14.2 rviz2: 3D Visualization

**rviz2** is the primary visualization tool for ROS 2:

```
┌─────────────────────────────────────────────────────────────────────┐
│                        rviz2 DISPLAYS                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Common Display Types:                                             │
│                                                                     │
│   ┌─────────────────┬────────────────────────────────────────┐     │
│   │ Display         │ Visualizes                              │     │
│   ├─────────────────┼────────────────────────────────────────┤     │
│   │ RobotModel      │ URDF-based robot visualization         │     │
│   │ TF              │ Coordinate frame axes                  │     │
│   │ LaserScan       │ 2D laser scan points                   │     │
│   │ PointCloud2     │ 3D point clouds (LiDAR, depth)         │     │
│   │ Image           │ Camera images                          │     │
│   │ Marker          │ Custom 3D markers                      │     │
│   │ Path            │ Navigation paths                       │     │
│   │ Odometry        │ Odometry arrows                        │     │
│   │ Map             │ 2D occupancy grid                      │     │
│   │ MarkerArray     │ Multiple markers                       │     │
│   └─────────────────┴────────────────────────────────────────┘     │
│                                                                     │
│   Configuration:                                                    │
│   • Fixed Frame: Reference frame for visualization                 │
│   • Topics: Select which topics to visualize                       │
│   • Colors/sizes: Customize appearance                             │
│   • Save/load .rviz config files                                   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

```bash
# Launch rviz2
ros2 run rviz2 rviz2

# Launch with config file
ros2 run rviz2 rviz2 -d config/robot.rviz
```

### 5.14.3 ros2 bag: Recording and Playback

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /scan /odom /tf -o my_recording

# Record with compression
ros2 bag record /scan /odom --compression-mode file

# List info about a bag
ros2 bag info my_recording

# Play back a bag
ros2 bag play my_recording

# Play at different speed
ros2 bag play my_recording --rate 2.0

# Play specific topics
ros2 bag play my_recording --topics /scan /odom

# Loop playback
ros2 bag play my_recording --loop
```

### 5.14.4 rqt Tools

```bash
# Main rqt GUI
rqt

# Specific tools
rqt_graph          # Visualize computation graph
rqt_topic          # Topic monitor/publisher
rqt_console        # Log viewer
rqt_plot           # Plot numeric values
rqt_image_view     # Image viewer
rqt_reconfigure    # Dynamic parameter editor
```

### 5.14.5 colcon Build Tool

```bash
# Build entire workspace
colcon build

# Build specific packages
colcon build --packages-select pkg1 pkg2

# Build with dependencies
colcon build --packages-up-to target_pkg

# Symlink install (faster Python development)
colcon build --symlink-install

# Parallel build jobs
colcon build --parallel-workers 4

# Clean build
rm -rf build/ install/ log/
colcon build

# Run tests
colcon test
colcon test-result --verbose
```

---

## 5.15 Mini-Projects

### 5.15.1 Project 1: Simple Teleoperation System

Build a complete teleoperation system with velocity limiting:

```python
#!/usr/bin/env python3
"""
teleop_system.py - Complete teleoperation system
Mini-Project 1: Keyboard teleoperation with velocity limiting
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sys
import termios
import tty


class TeleopKeyboard(Node):
    """Keyboard teleoperation node."""

    KEYS = {
        'w': (1.0, 0.0),   # Forward
        's': (-1.0, 0.0),  # Backward
        'a': (0.0, 1.0),   # Turn left
        'd': (0.0, -1.0),  # Turn right
        'q': (1.0, 1.0),   # Forward-left
        'e': (1.0, -1.0),  # Forward-right
        ' ': (0.0, 0.0),   # Stop
    }

    def __init__(self):
        super().__init__('teleop_keyboard')

        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.0)

        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value

        self.publisher = self.create_publisher(Twist, 'cmd_vel_raw', 10)
        self.enabled_pub = self.create_publisher(Bool, 'teleop_enabled', 10)

        self.get_logger().info(
            f'Teleop ready. WASD to move, SPACE to stop, Q to quit.'
        )

    def get_key(self):
        """Get single keypress."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def run(self):
        """Main teleop loop."""
        try:
            while True:
                key = self.get_key().lower()

                if key == '\x03' or key == 'x':  # Ctrl+C or x
                    break

                if key in self.KEYS:
                    linear_scale, angular_scale = self.KEYS[key]

                    msg = Twist()
                    msg.linear.x = linear_scale * self.max_linear
                    msg.angular.z = angular_scale * self.max_angular

                    self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # Send stop command
            self.publisher.publish(Twist())


class VelocityLimiter(Node):
    """Limits and smooths velocity commands."""

    def __init__(self):
        super().__init__('velocity_limiter')

        self.declare_parameter('max_linear', 1.0)
        self.declare_parameter('max_angular', 2.0)
        self.declare_parameter('linear_accel', 0.5)
        self.declare_parameter('angular_accel', 1.0)

        self.max_lin = self.get_parameter('max_linear').value
        self.max_ang = self.get_parameter('max_angular').value
        self.lin_accel = self.get_parameter('linear_accel').value
        self.ang_accel = self.get_parameter('angular_accel').value

        self.current_linear = 0.0
        self.current_angular = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0

        self.subscription = self.create_subscription(
            Twist, 'cmd_vel_raw', self.cmd_callback, 10
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.02, self.update)

    def cmd_callback(self, msg):
        self.target_linear = max(-self.max_lin,
                                 min(self.max_lin, msg.linear.x))
        self.target_angular = max(-self.max_ang,
                                  min(self.max_ang, msg.angular.z))

    def update(self):
        dt = 0.02

        # Apply acceleration limits
        lin_diff = self.target_linear - self.current_linear
        max_lin_change = self.lin_accel * dt
        self.current_linear += max(-max_lin_change,
                                   min(max_lin_change, lin_diff))

        ang_diff = self.target_angular - self.current_angular
        max_ang_change = self.ang_accel * dt
        self.current_angular += max(-max_ang_change,
                                    min(max_ang_change, ang_diff))

        msg = Twist()
        msg.linear.x = self.current_linear
        msg.angular.z = self.current_angular
        self.publisher.publish(msg)
```

### 5.15.2 Project 2: Sensor Fusion Node

Combine IMU and odometry for improved state estimation:

```python
#!/usr/bin/env python3
"""
sensor_fusion.py - Simple complementary filter fusion
Mini-Project 2: Fuse IMU orientation with wheel odometry
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import math


class SensorFusion(Node):
    """
    Fuses IMU and odometry using complementary filter.

    - Position: Pure odometry (no IMU position)
    - Orientation: Complementary filter (IMU + odom)
    """

    def __init__(self):
        super().__init__('sensor_fusion')

        # Filter parameter (0-1, higher = more IMU trust)
        self.declare_parameter('alpha', 0.98)
        self.alpha = self.get_parameter('alpha').value

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = None

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, 'imu', self.imu_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )

        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped, 'fused_pose', 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        # Store latest IMU yaw rate
        self.imu_yaw_rate = 0.0

        self.get_logger().info(f'Sensor fusion started (alpha={self.alpha})')

    def imu_callback(self, msg: Imu):
        """Extract yaw rate from IMU."""
        self.imu_yaw_rate = msg.angular_velocity.z

    def odom_callback(self, msg: Odometry):
        """Process odometry and fuse with IMU."""
        current_time = self.get_clock().now()

        if self.last_odom_time is None:
            self.last_odom_time = current_time
            return

        dt = (current_time - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = current_time

        # Extract odometry velocities
        vx = msg.twist.twist.linear.x
        wz_odom = msg.twist.twist.angular.z

        # Complementary filter for yaw rate
        wz_fused = self.alpha * self.imu_yaw_rate + (1 - self.alpha) * wz_odom

        # Integrate pose
        self.theta += wz_fused * dt
        self.x += vx * math.cos(self.theta) * dt
        self.y += vx * math.sin(self.theta) * dt

        # Publish fused pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time.to_msg()
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.orientation.z = math.sin(self.theta / 2)
        pose_msg.pose.orientation.w = math.cos(self.theta / 2)
        self.pose_pub.publish(pose_msg)

        # Broadcast TF
        t = TransformStamped()
        t.header = pose_msg.header
        t.child_frame_id = 'base_link_fused'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 5.15.3 Project 3: Service-Action Waypoint Navigator

Combine services and actions for waypoint navigation:

```python
#!/usr/bin/env python3
"""
waypoint_navigator.py - Waypoint navigation with services and actions
Mini-Project 3: Service to add waypoints, action to navigate
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Path
from std_srvs.srv import Empty, Trigger
from example_interfaces.action import Fibonacci  # Placeholder
import math
from collections import deque


class WaypointNavigator(Node):
    """
    Waypoint navigation system.

    Services:
        /add_waypoint (AddWaypoint): Add waypoint to queue
        /clear_waypoints (Empty): Clear waypoint queue
        /get_waypoint_count (Trigger): Get number of queued waypoints

    Topics:
        /waypoints (Path): Visualization of waypoint queue
        /cmd_vel (Twist): Velocity commands
    """

    def __init__(self):
        super().__init__('waypoint_navigator')

        self.callback_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('goal_tolerance', 0.1)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # State
        self.waypoints = deque()
        self.current_pose = PoseStamped()
        self.navigating = False

        # Services
        self.clear_srv = self.create_service(
            Empty, 'clear_waypoints', self.clear_waypoints_callback
        )
        self.count_srv = self.create_service(
            Trigger, 'get_waypoint_count', self.get_count_callback
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, 'waypoints', 10)

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, 'robot_pose', self.pose_callback, 10
        )

        # Navigation timer
        self.nav_timer = self.create_timer(0.1, self.navigation_loop)

        self.get_logger().info('Waypoint navigator ready')

    def clear_waypoints_callback(self, request, response):
        self.waypoints.clear()
        self.navigating = False
        self.cmd_pub.publish(Twist())  # Stop robot
        self.get_logger().info('Waypoints cleared')
        return response

    def get_count_callback(self, request, response):
        response.success = True
        response.message = f'{len(self.waypoints)} waypoints queued'
        return response

    def pose_callback(self, msg):
        self.current_pose = msg

    def add_waypoint(self, x, y):
        """Add waypoint to queue."""
        self.waypoints.append(Point(x=x, y=y, z=0.0))
        self.publish_path()
        if not self.navigating:
            self.navigating = True

    def publish_path(self):
        """Publish waypoint path for visualization."""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position = wp
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.path_pub.publish(path)

    def navigation_loop(self):
        """Main navigation control loop."""
        if not self.navigating or not self.waypoints:
            return

        target = self.waypoints[0]

        # Compute distance and angle to target
        dx = target.x - self.current_pose.pose.position.x
        dy = target.y - self.current_pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance < self.goal_tolerance:
            self.waypoints.popleft()
            self.publish_path()
            self.get_logger().info(
                f'Reached waypoint. {len(self.waypoints)} remaining.'
            )
            if not self.waypoints:
                self.navigating = False
                self.cmd_pub.publish(Twist())
            return

        # Simple proportional control
        target_angle = math.atan2(dy, dx)

        # Get current yaw from quaternion
        q = self.current_pose.pose.orientation
        current_yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                                  1 - 2*(q.y*q.y + q.z*q.z))

        angle_error = target_angle - current_yaw
        # Normalize to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2*math.pi
        while angle_error < -math.pi:
            angle_error += 2*math.pi

        # Generate velocity command
        cmd = Twist()

        if abs(angle_error) > 0.3:  # Turn first
            cmd.angular.z = self.angular_speed * (1 if angle_error > 0 else -1)
        else:
            cmd.linear.x = min(self.linear_speed, distance)
            cmd.angular.z = 2.0 * angle_error

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 5.16 Chapter Summary

### Key Concepts Mastered

This chapter provided a comprehensive foundation in ROS 2, covering:

1. **Architecture**: ROS 2's node-based computation graph and DDS middleware
2. **Communication**: Topics (pub/sub), Services (request/response), Actions (goal-oriented)
3. **Quality of Service**: Configuring reliability, durability, and history for different use cases
4. **rclpy**: Writing Python nodes with timers, callbacks, and proper lifecycle management
5. **Packages**: Creating, organizing, and building ROS 2 packages with colcon
6. **Launch Files**: Composing complex systems with Python launch files
7. **Parameters**: Runtime configuration with validation and callbacks
8. **URDF**: Describing robot structure for visualization and simulation
9. **ros2_control**: Framework overview for hardware abstraction
10. **Lifecycle Nodes**: Managed state transitions for deterministic behavior
11. **TF2**: Coordinate frame transformations for spatial reasoning
12. **Tools**: rviz2, ros2 bag, rqt, and CLI commands

### Connection to Physical AI

ROS 2 serves as the software backbone for Physical AI systems:

```
┌─────────────────────────────────────────────────────────────────────┐
│              ROS 2 IN THE PHYSICAL AI STACK                         │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌─────────────────────────────────────────────────────────────┐  │
│   │                    AI/ML LAYER                               │  │
│   │     Neural Networks │ Planning │ Decision Making            │  │
│   └───────────────────────────┬─────────────────────────────────┘  │
│                               │                                     │
│   ┌───────────────────────────▼─────────────────────────────────┐  │
│   │                    ROS 2 LAYER                               │  │
│   │   Communication │ Coordination │ Hardware Abstraction        │  │
│   └───────────────────────────┬─────────────────────────────────┘  │
│                               │                                     │
│   ┌───────────────────────────▼─────────────────────────────────┐  │
│   │                  HARDWARE LAYER                              │  │
│   │     Sensors (Ch.2) │ Actuators (Ch.3) │ Controllers (Ch.4)  │  │
│   └─────────────────────────────────────────────────────────────┘  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### What's Next

Chapter 6 will introduce **Simulation with Gazebo**, where we'll:
- Build complete robot simulations using the URDF concepts from this chapter
- Integrate simulated sensors that publish to ROS 2 topics
- Test control systems developed in Chapter 4 in a safe virtual environment
- Prepare for real hardware deployment with sim-to-real workflows

---

## 5.17 Glossary

| Term | Definition |
|------|------------|
| **Action** | ROS 2 communication pattern for long-running, preemptable tasks with feedback |
| **ament** | ROS 2 build system, replacing catkin from ROS 1 |
| **Callback** | Function executed when an event occurs (message received, timer fires) |
| **colcon** | Command-line tool for building ROS 2 workspaces |
| **Computation Graph** | Network of nodes and their communication connections |
| **DDS** | Data Distribution Service, the middleware standard underlying ROS 2 |
| **Durability** | QoS policy controlling message persistence for late joiners |
| **Executor** | Schedules and executes callbacks in a node |
| **Frame** | A coordinate system in the TF2 tree |
| **History** | QoS policy controlling how many messages are stored |
| **Launch File** | Python script defining how to start multiple nodes |
| **Lifecycle Node** | Node with managed state transitions (configure, activate, etc.) |
| **Message** | Typed data structure for communication |
| **Middleware** | Software layer enabling communication between distributed components |
| **Namespace** | Prefix for node/topic names enabling organization and isolation |
| **Node** | Fundamental unit of computation in ROS 2 |
| **Package** | Unit of code organization containing nodes, libraries, configs |
| **Parameter** | Runtime configuration value for a node |
| **Publisher** | Entity that sends messages to a topic |
| **QoS** | Quality of Service, policies controlling communication behavior |
| **rclpy** | ROS Client Library for Python |
| **Reliability** | QoS policy choosing between guaranteed or best-effort delivery |
| **Remapping** | Changing topic/service/parameter names at runtime |
| **ROS 2** | Robot Operating System 2, middleware framework for robotics |
| **Service** | Synchronous request-response communication pattern |
| **spin** | Process that keeps a node running and processing callbacks |
| **Static Transform** | Fixed coordinate frame relationship (doesn't change over time) |
| **Subscriber** | Entity that receives messages from a topic |
| **TF2** | Transform library for coordinate frame management |
| **Timer** | Triggers callbacks at regular intervals |
| **Topic** | Named channel for publish-subscribe communication |
| **Transform** | Position and orientation relationship between coordinate frames |
| **URDF** | Unified Robot Description Format for robot modeling |
| **Workspace** | Directory containing ROS 2 packages for development |
| **xacro** | XML macro language for URDF templates |

---

## 5.18 Review Questions and Exercises

### Conceptual Questions

1. **Communication Patterns**: Explain when you would use a Topic vs. a Service vs. an Action. Provide an example use case for each.

2. **QoS Understanding**: A subscriber with `RELIABLE` QoS cannot receive messages from a publisher with `BEST_EFFORT` QoS. Why? How would you diagnose this issue?

3. **Lifecycle Benefits**: What advantages do lifecycle nodes provide over regular nodes? When would you choose to implement a lifecycle node?

4. **DDS Discovery**: Unlike ROS 1, ROS 2 doesn't require a master node. How does node discovery work in ROS 2?

5. **TF2 Design**: Why does TF2 maintain a tree structure rather than allowing arbitrary frame relationships?

### Practical Exercises

**Exercise 1: Multi-Sensor Publisher**

Create a package containing:
- A node that publishes simulated IMU data at 100 Hz
- A node that publishes simulated GPS data at 1 Hz
- A launch file that starts both with configurable parameters
- Appropriate QoS settings for each sensor type

**Exercise 2: Parameter-Configurable PID**

Implement a PID controller node that:
- Subscribes to `/error` (Float64)
- Publishes to `/control_output` (Float64)
- Has parameters for Kp, Ki, Kd with dynamic reconfiguration
- Validates that gains are non-negative
- Logs when parameters change

**Exercise 3: Transform Chain**

Create a system that:
- Broadcasts static transforms: map → odom → base_link → sensor_link
- Implements a node that listens for sensor_link → map transform
- Publishes the sensor position in the map frame
- Handles lookup failures gracefully

**Exercise 4: Service-Based Configuration**

Create a "robot mode manager" node with:
- Service `/set_mode` that accepts modes: "idle", "manual", "autonomous"
- Publisher for current mode (String)
- Different behavior/logging based on current mode
- Rejection of invalid mode requests

**Exercise 5: Complete Robot System**

Integrate concepts into a complete differential drive robot simulation:
1. Create URDF for a two-wheeled robot with a caster
2. Write a node that publishes wheel joint states
3. Create launch file starting robot_state_publisher
4. Visualize in rviz2 with robot model and TF frames
5. Add keyboard teleop controlling wheel velocities

### Programming Challenges

**Challenge 1: Message Latency Monitor**

Create a diagnostic node that:
- Subscribes to any topic with timestamped messages
- Computes and publishes latency statistics (min, max, mean, std)
- Warns if latency exceeds a configurable threshold
- Uses lifecycle management for clean startup/shutdown

**Challenge 2: Topic Recorder**

Implement a simplified version of ros2 bag:
- Service to start/stop recording
- Parameter for output filename
- Record specified topics to a CSV or JSON file
- Handle multiple message types

**Challenge 3: Distributed Computation**

Create a multi-node computation pipeline:
- Node A: Generates random numbers, publishes to /raw_data
- Node B: Subscribes to /raw_data, computes running average, publishes to /filtered_data
- Node C: Subscribes to /filtered_data, logs when value exceeds threshold
- Launch file starting all three with namespace support
- Demonstrate running across multiple terminals

---

*This chapter establishes the software infrastructure for Physical AI development. With ROS 2 fundamentals mastered, you now have the tools to integrate the hardware concepts from previous chapters into complete robotic systems. The simulation chapter that follows will provide a safe environment to practice these skills before deploying to real robots.*
