---
id: ch6-digital-twin-simulation
title: "Chapter 6: Digital Twin & Simulation"
sidebar_label: "6. Digital Twin & Simulation"
sidebar_position: 6
---

# Chapter 6: Digital Twin & Simulation — Gazebo + Unity

## 6.1 Introduction to Digital Twin

Before deploying a Physical AI system in the real world, we need a safe space to test, iterate, and validate our designs. A single hardware failure during testing can cost thousands of dollars and weeks of repair time. A software bug in a walking humanoid can result in catastrophic falls. This is where **simulation** becomes indispensable.

Simulation provides a virtual environment where robots can fail safely, learn from mistakes, and be tested under conditions that would be dangerous or impossible to recreate physically. But modern robotics demands more than simple visualization—it requires **Digital Twins**: synchronized virtual replicas that mirror real-world physics, sensors, and behaviors with high fidelity.

### Why Simulation Matters for Physical AI

| Challenge | Real Hardware | Simulation |
|-----------|---------------|------------|
| Cost of failure | Expensive repairs | Zero cost |
| Testing speed | Real-time only | Faster than real-time |
| Reproducibility | Environmental variations | Exact repeatability |
| Edge cases | Dangerous to test | Safe exploration |
| Parallel testing | Limited by hardware | Unlimited instances |
| Sensor data | Hardware required | Synthetic generation |

### Chapter Learning Objectives

By the end of this chapter, you will:

1. Understand Digital Twin concepts and their role in Physical AI development
2. Create Gazebo simulation worlds with realistic physics
3. Simulate sensors (LiDAR, cameras, IMU) with noise models
4. Convert URDF robot descriptions to Gazebo-compatible SDF
5. Integrate ros2_control with simulated hardware
6. Build high-fidelity visualizations using Unity (Part B)
7. Complete a mini-project: Virtual lab with moving humanoid joints

---

## 6.2 What is a Digital Twin in Physical AI?

A **Digital Twin** is a virtual representation of a physical system that maintains bidirectional synchronization with its real-world counterpart. Unlike simple simulations, Digital Twins are:

- **Synchronized**: Real-time data flows between physical and virtual
- **Persistent**: The virtual model evolves with the physical system
- **Predictive**: Can forecast behavior before physical execution
- **Validated**: Continuously calibrated against real measurements

```
┌─────────────────────────────────────────────────────────────────────┐
│                    DIGITAL TWIN ARCHITECTURE                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌─────────────────┐              ┌─────────────────┐             │
│   │  PHYSICAL ROBOT │◄────────────►│  DIGITAL TWIN   │             │
│   │                 │   Real-time  │                 │             │
│   │  • Sensors      │     Sync     │  • Virtual      │             │
│   │  • Actuators    │◄────────────►│    Sensors      │             │
│   │  • Environment  │              │  • Physics Sim  │             │
│   │                 │              │  • Prediction   │             │
│   └────────┬────────┘              └────────┬────────┘             │
│            │                                │                       │
│            ▼                                ▼                       │
│   ┌─────────────────────────────────────────────────────┐          │
│   │                   ROS 2 MIDDLEWARE                   │          │
│   │  Topics: /joint_states, /cmd_vel, /scan, /camera    │          │
│   └─────────────────────────────────────────────────────┘          │
│                              │                                      │
│                              ▼                                      │
│   ┌─────────────────────────────────────────────────────┐          │
│   │                  AI / CONTROL LAYER                  │          │
│   │     Planning │ Learning │ Control │ Perception       │          │
│   └─────────────────────────────────────────────────────┘          │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Digital Twin vs Traditional Simulation

| Aspect | Traditional Simulation | Digital Twin |
|--------|----------------------|--------------|
| Data flow | One-way (model → sim) | Bidirectional |
| Synchronization | None | Real-time |
| Lifecycle | Single use | Persistent |
| Calibration | Manual, initial | Continuous |
| Purpose | Design validation | Operational insight |

### The Simulation-Reality Gap

Even the best simulations differ from reality. This **sim-to-real gap** arises from:

- **Physics approximations**: Real contact dynamics are complex
- **Sensor noise**: Real sensors have imperfections simulations may miss
- **Unmodeled dynamics**: Cables, friction variations, temperature effects
- **Environmental factors**: Lighting, air currents, floor surfaces

Minimizing this gap is critical for Physical AI systems that must transfer learned behaviors from simulation to reality.

---

## 6.3 Gazebo Overview

**Gazebo** is the most widely used robotics simulator, tightly integrated with ROS 2. It provides physics simulation, sensor modeling, and 3D visualization in one package.

### Gazebo Classic vs Gazebo Sim (Ignition)

| Feature | Gazebo Classic (11) | Gazebo Sim (Fortress/Harmonic) |
|---------|--------------------|---------------------------------|
| Architecture | Monolithic | Modular (libraries) |
| Physics engines | ODE, Bullet, DART, Simbody | DART, TPE, Bullet |
| ROS 2 integration | gazebo_ros_pkgs | ros_gz (gz-sim) |
| GUI | Qt-based | Modern Qt/QML |
| Performance | Good | Better (multi-threaded) |
| Active development | Maintenance only | Active |
| Recommended for | Legacy projects | New projects |

**This textbook uses Gazebo Fortress (LTS)** for ROS 2 Humble compatibility, but concepts apply to both versions.

```
┌─────────────────────────────────────────────────────────────────────┐
│                    GAZEBO ARCHITECTURE                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌─────────────────────────────────────────────────────────────┐  │
│   │                      GAZEBO SERVER                           │  │
│   │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐            │  │
│   │  │   World     │ │   Physics   │ │   Sensors   │            │  │
│   │  │   Manager   │ │   Engine    │ │   Manager   │            │  │
│   │  └──────┬──────┘ └──────┬──────┘ └──────┬──────┘            │  │
│   │         │               │               │                    │  │
│   │         └───────────────┼───────────────┘                    │  │
│   │                         ▼                                    │  │
│   │              ┌─────────────────────┐                        │  │
│   │              │    Transport Layer   │                        │  │
│   │              │   (gz-transport)     │                        │  │
│   │              └──────────┬──────────┘                        │  │
│   └─────────────────────────┼───────────────────────────────────┘  │
│                             │                                       │
│   ┌─────────────────────────▼───────────────────────────────────┐  │
│   │                      GAZEBO CLIENT                           │  │
│   │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐            │  │
│   │  │   3D View   │ │   Plugins   │ │  ros_gz     │            │  │
│   │  │   (GUI)     │ │   (Custom)  │ │  bridge     │            │  │
│   │  └─────────────┘ └─────────────┘ └─────────────┘            │  │
│   └─────────────────────────────────────────────────────────────┘  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 6.4 Gazebo World Files (SDF)

Gazebo uses **SDF (Simulation Description Format)** to define worlds, models, and sensors. SDF is more expressive than URDF, supporting:

- Multiple robots and objects
- Lights and atmospheric effects
- Physics engine configuration
- Sensor plugins
- Actor animations

### Basic World Structure

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="physical_ai_lab">

    <!-- Physics Configuration -->
    <physics type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Gravity -->
    <gravity>0 0 -9.81</gravity>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>
                <mu2>0.8</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Include Robot Model -->
    <include>
      <uri>model://my_robot</uri>
      <pose>0 0 0.5 0 0 0</pose>
    </include>

  </world>
</sdf>
```

### Key SDF Elements

| Element | Purpose | Example Values |
|---------|---------|----------------|
| `<physics>` | Engine configuration | dart, ode, bullet |
| `<gravity>` | World gravity vector | `0 0 -9.81` |
| `<light>` | Scene illumination | directional, point, spot |
| `<model>` | Physical object | robot, obstacle, ground |
| `<static>` | Immovable object | true/false |
| `<pose>` | Position + orientation | `x y z roll pitch yaw` |

---

## 6.5 Physics Engine Configuration

Gazebo's physics engine simulates rigid body dynamics, collisions, and contact forces. Proper configuration is critical for realistic simulation.

### Physics Parameters

```xml
<physics type="dart">
  <!-- Simulation timestep (smaller = more accurate, slower) -->
  <max_step_size>0.001</max_step_size>

  <!-- Real-time factor (1.0 = real-time, 2.0 = 2x speed) -->
  <real_time_factor>1.0</real_time_factor>

  <!-- Physics updates per second -->
  <real_time_update_rate>1000</real_time_update_rate>

  <!-- DART-specific settings -->
  <dart>
    <collision_detector>bullet</collision_detector>
    <solver>
      <solver_type>pgs</solver_type>
      <pgs_iterations>50</pgs_iterations>
    </solver>
  </dart>
</physics>
```

### Collision and Friction

```xml
<collision name="box_collision">
  <geometry>
    <box><size>1 1 1</size></box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>0.8</mu>      <!-- Primary friction coefficient -->
        <mu2>0.6</mu2>    <!-- Secondary friction coefficient -->
        <slip1>0.0</slip1>
        <slip2>0.0</slip2>
      </ode>
    </friction>
    <contact>
      <ode>
        <kp>1e6</kp>      <!-- Contact stiffness -->
        <kd>100</kd>      <!-- Contact damping -->
        <max_vel>0.1</max_vel>
        <min_depth>0.001</min_depth>
      </ode>
    </contact>
    <bounce>
      <restitution_coefficient>0.1</restitution_coefficient>
      <threshold>0.01</threshold>
    </bounce>
  </surface>
</collision>
```

### Inertial Properties

Accurate inertia is critical for realistic dynamics:

```xml
<inertial>
  <mass>5.0</mass>
  <pose>0 0 0 0 0 0</pose>  <!-- Center of mass -->
  <inertia>
    <ixx>0.0833</ixx>  <!-- I = (1/12) * m * (h² + d²) for box -->
    <ixy>0.0</ixy>
    <ixz>0.0</ixz>
    <iyy>0.0833</iyy>
    <iyz>0.0</iyz>
    <izz>0.0833</izz>
  </inertia>
</inertial>
```

**Inertia formulas for common shapes:**

| Shape | Ixx, Iyy, Izz |
|-------|---------------|
| Box (w×h×d) | m(h²+d²)/12, m(w²+d²)/12, m(w²+h²)/12 |
| Cylinder (r, h) | m(3r²+h²)/12, m(3r²+h²)/12, mr²/2 |
| Sphere (r) | 2mr²/5, 2mr²/5, 2mr²/5 |

---

## 6.6 Sensor Simulation in Gazebo

Gazebo can simulate various sensors with configurable noise models, enabling realistic perception testing.

### 6.6.1 LiDAR Sensor

```xml
<sensor name="lidar" type="gpu_lidar">
  <pose>0 0 0.2 0 0 0</pose>
  <topic>/scan</topic>
  <update_rate>10</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.02</stddev>
    </noise>
  </lidar>
  <always_on>true</always_on>
  <visualize>true</visualize>
</sensor>
```

### 6.6.2 RGB Camera

```xml
<sensor name="camera" type="camera">
  <pose>0.1 0 0.2 0 0 0</pose>
  <topic>/camera/image_raw</topic>
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <always_on>true</always_on>
</sensor>
```

### 6.6.3 Depth Camera

```xml
<sensor name="depth_camera" type="depth_camera">
  <pose>0.1 0 0.2 0 0 0</pose>
  <topic>/depth/image_raw</topic>
  <update_rate>15</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R_FLOAT32</format>
    </image>
    <clip>
      <near>0.3</near>
      <far>10.0</far>
    </clip>
  </camera>
  <always_on>true</always_on>
</sensor>
```

### 6.6.4 IMU Sensor

```xml
<sensor name="imu" type="imu">
  <pose>0 0 0.1 0 0 0</pose>
  <topic>/imu/data</topic>
  <update_rate>200</update_rate>
  <imu>
    <angular_velocity>
      <x><noise type="gaussian">
        <mean>0.0</mean>
        <stddev>0.0002</stddev>
        <bias_mean>0.0000075</bias_mean>
        <bias_stddev>0.0000008</bias_stddev>
      </noise></x>
      <y><noise type="gaussian">
        <mean>0.0</mean>
        <stddev>0.0002</stddev>
      </noise></y>
      <z><noise type="gaussian">
        <mean>0.0</mean>
        <stddev>0.0002</stddev>
      </noise></z>
    </angular_velocity>
    <linear_acceleration>
      <x><noise type="gaussian">
        <mean>0.0</mean>
        <stddev>0.017</stddev>
      </noise></x>
      <y><noise type="gaussian">
        <mean>0.0</mean>
        <stddev>0.017</stddev>
      </noise></y>
      <z><noise type="gaussian">
        <mean>0.0</mean>
        <stddev>0.017</stddev>
      </noise></z>
    </linear_acceleration>
  </imu>
  <always_on>true</always_on>
</sensor>
```

### 6.6.5 Noise Models Summary

| Sensor | Noise Type | Typical Parameters |
|--------|------------|-------------------|
| LiDAR | Gaussian | σ = 0.01-0.05 m |
| Camera | Gaussian (pixel) | σ = 0.005-0.01 |
| Depth | Gaussian | σ = 0.01-0.03 m |
| IMU Gyro | Gaussian + Bias | σ = 0.0002 rad/s |
| IMU Accel | Gaussian + Bias | σ = 0.017 m/s² |

---

## 6.7 URDF to SDF Conversion

URDF (from Chapter 5) describes robot structure but lacks simulation features. Gazebo extends URDF with `<gazebo>` tags or converts to SDF.

### URDF with Gazebo Extensions

```xml
<?xml version="1.0"?>
<robot name="simple_arm" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry><cylinder radius="0.1" length="0.05"/></geometry>
      <material name="gray"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
    <collision>
      <geometry><cylinder radius="0.1" length="0.05"/></geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Gazebo-specific: Material color -->
  <gazebo reference="base_link">
    <material>Gazebo/Grey</material>
  </gazebo>

  <!-- Arm Link -->
  <link name="arm_link">
    <visual>
      <origin xyz="0 0 0.15"/>
      <geometry><box size="0.05 0.05 0.3"/></geometry>
      <material name="blue"><color rgba="0 0 0.8 1"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 0.15"/>
      <geometry><box size="0.05 0.05 0.3"/></geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.15"/>
      <mass value="1.0"/>
      <inertia ixx="0.008" ixy="0" ixz="0" iyy="0.008" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <gazebo reference="arm_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <!-- Revolute Joint -->
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.025"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <!-- Gazebo Joint Properties -->
  <gazebo reference="arm_joint">
    <implicitSpringDamper>true</implicitSpringDamper>
  </gazebo>

  <!-- Gazebo ros2_control Plugin -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find my_robot)/config/controllers.yaml</parameters>
    </plugin>
  </gazebo>

</robot>
```

### SDF vs URDF Comparison

| Feature | URDF | SDF |
|---------|------|-----|
| Purpose | Robot description | World + models |
| Multiple models | No | Yes |
| Sensor definitions | Via plugins | Native support |
| Closed kinematic chains | No | Yes |
| Physics configuration | No | Yes |
| Lights, actors | No | Yes |
| ROS integration | Native | Via ros_gz bridge |

**Conversion command:**
```bash
# URDF to SDF conversion
gz sdf -p robot.urdf > robot.sdf
```

---

## 6.8 Gazebo Plugins

Plugins extend Gazebo functionality for custom behaviors, sensor processing, and ROS integration.

### Common Plugin Types

| Plugin Type | Purpose | Example |
|-------------|---------|---------|
| Model | Control models | Differential drive |
| World | Modify world | Object spawner |
| Sensor | Process sensor data | Camera publisher |
| System | Core functionality | Physics engine |

### Differential Drive Plugin Example

```xml
<plugin filename="libgazebo_ros_diff_drive.so" name="diff_drive">
  <ros>
    <namespace>/robot</namespace>
    <remapping>cmd_vel:=cmd_vel</remapping>
    <remapping>odom:=odom</remapping>
  </ros>

  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>

  <wheel_separation>0.4</wheel_separation>
  <wheel_diameter>0.2</wheel_diameter>

  <max_wheel_torque>20</max_wheel_torque>
  <max_wheel_acceleration>1.0</max_wheel_acceleration>

  <publish_odom>true</publish_odom>
  <publish_odom_tf>true</publish_odom_tf>
  <publish_wheel_tf>false</publish_wheel_tf>

  <odometry_frame>odom</odometry_frame>
  <robot_base_frame>base_link</robot_base_frame>
</plugin>
```

---

## 6.9 ros2_control in Simulation

Integrating ros2_control with Gazebo provides standardized control interfaces that work identically in simulation and on real hardware.

### Controller Configuration (YAML)

```yaml
# config/controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: position_controllers/JointGroupPositionController

arm_controller:
  ros__parameters:
    joints:
      - arm_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

### Launch File for Gazebo + ros2_control

```python
# launch/simulation.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('my_robot').find('my_robot')

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros').find('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={'world': os.path.join(pkg_share, 'worlds', 'lab.sdf')}.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )

    # Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(os.path.join(pkg_share, 'urdf', 'robot.urdf')).read()}]
    )

    # Controller Manager
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 'arm_controller'],
    )

    return LaunchDescription([
        gazebo,
        robot_state_pub,
        spawn_robot,
        controller_spawner,
    ])
```

### URDF ros2_control Hardware Interface

```xml
<ros2_control name="GazeboSimSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>

  <joint name="arm_joint">
    <command_interface name="position">
      <param name="min">-1.57</param>
      <param name="max">1.57</param>
    </command_interface>
    <command_interface name="velocity">
      <param name="min">-1.0</param>
      <param name="max">1.0</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

### Testing the Simulation

```bash
# Terminal 1: Launch simulation
ros2 launch my_robot simulation.launch.py

# Terminal 2: Send position command
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5]"

# Terminal 3: Monitor joint states
ros2 topic echo /joint_states
```

---

## 6.10 Why Unity for Robotics?

While Gazebo excels at physics simulation and ROS integration, **Unity** brings capabilities essential for modern Physical AI development:

- **Photorealistic Rendering**: PBR materials, global illumination, ray tracing
- **Domain Randomization**: Vary textures, lighting, and objects for robust ML training
- **Human-Robot Interaction**: Animate humans alongside robots
- **Cross-Platform Deployment**: Windows, Linux, VR/AR headsets
- **Large Asset Ecosystem**: Thousands of ready-made 3D models

### When to Use Unity vs Gazebo

| Requirement | Best Choice |
|-------------|-------------|
| ROS 2 native integration | Gazebo |
| Physics accuracy for control | Gazebo |
| Photorealistic visuals | Unity |
| ML perception training | Unity |
| VR/AR visualization | Unity |
| Human animation | Unity |
| Quick prototyping | Gazebo |
| Production deployment | Both |

**Recommendation**: Use Gazebo for control development, Unity for perception and visualization. Connect both via ROS 2 bridge for complete Digital Twin.

---

## 6.11 Unity Robotics Hub Overview

Unity provides the **Unity Robotics Hub** - a collection of packages for robotics development:

```
┌─────────────────────────────────────────────────────────────────────┐
│                  UNITY ROBOTICS STACK                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌─────────────────────────────────────────────────────────────┐  │
│   │                    UNITY EDITOR                              │  │
│   │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐            │  │
│   │  │   Scene     │ │  Hierarchy  │ │  Inspector  │            │  │
│   │  │   View      │ │   Panel     │ │   Panel     │            │  │
│   │  └─────────────┘ └─────────────┘ └─────────────┘            │  │
│   └─────────────────────────────────────────────────────────────┘  │
│                              │                                      │
│   ┌──────────────────────────▼──────────────────────────────────┐  │
│   │                 ROBOTICS PACKAGES                            │  │
│   │  ┌────────────┐ ┌────────────┐ ┌────────────┐               │  │
│   │  │   URDF     │ │ ROS-TCP    │ │ Navigation │               │  │
│   │  │  Importer  │ │ Connector  │ │  Sensors   │               │  │
│   │  └────────────┘ └────────────┘ └────────────┘               │  │
│   └─────────────────────────────────────────────────────────────┘  │
│                              │                                      │
│   ┌──────────────────────────▼──────────────────────────────────┐  │
│   │                   ROS 2 NETWORK                              │  │
│   │         TCP/IP Connection to ROS 2 Nodes                     │  │
│   └─────────────────────────────────────────────────────────────┘  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Key Packages

| Package | Purpose |
|---------|---------|
| **URDF Importer** | Import robot models from URDF files |
| **ROS TCP Connector** | Bidirectional ROS 2 communication |
| **Perception** | Synthetic data generation for ML |
| **Navigation** | Sensor simulation (LiDAR, cameras) |

### Installation

```bash
# In Unity Package Manager, add packages by git URL:
# https://github.com/Unity-Technologies/ROS-TCP-Connector.git
# https://github.com/Unity-Technologies/URDF-Importer.git
```

---

## 6.12 High-Fidelity Rendering and Physics

### Rendering Pipeline

Unity offers multiple rendering pipelines:

| Pipeline | Use Case | Performance |
|----------|----------|-------------|
| Built-in | Legacy, simple scenes | Fast |
| URP (Universal) | Cross-platform, mobile | Balanced |
| HDRP (High Definition) | Photorealism, PC/Console | Heavy |

**For robotics simulation**, URP provides the best balance of quality and performance.

### Physics Engine

Unity uses **PhysX** (NVIDIA) for rigid body dynamics:

```csharp
// Unity C# - Configure physics settings
using UnityEngine;

public class PhysicsConfig : MonoBehaviour
\{
    void Start()
    \{
        // Set physics timestep (similar to Gazebo max_step_size)
        Time.fixedDeltaTime = 0.001f;  // 1ms = 1000 Hz

        // Configure gravity
        Physics.gravity = new Vector3(0, -9.81f, 0);

        // Set solver iterations for accuracy
        Physics.defaultSolverIterations = 10;
        Physics.defaultSolverVelocityIterations = 10;
    \}
\}
```

### Articulation Bodies for Robots

Unity's **ArticulationBody** component is designed for robotic chains:

```csharp
// Configure a robot joint
using UnityEngine;

public class JointController : MonoBehaviour
\{
    private ArticulationBody joint;

    void Start()
    \{
        joint = GetComponent&lt;ArticulationBody&gt;();

        // Configure joint drive
        var drive = joint.xDrive;
        drive.stiffness = 10000f;   // Position gain
        drive.damping = 100f;       // Velocity damping
        drive.forceLimit = 1000f;   // Max torque
        joint.xDrive = drive;
    \}

    public void SetTargetPosition(float angle)
    \{
        var drive = joint.xDrive;
        drive.target = angle * Mathf.Rad2Deg;
        joint.xDrive = drive;
    \}
\}
```

---

## 6.13 ROS-Unity Integration

### ROS TCP Connector Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                  ROS-UNITY DATA FLOW                                │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌─────────────────┐         TCP/IP        ┌─────────────────┐    │
│   │                 │◄─────────────────────►│                 │    │
│   │  UNITY SCENE    │                       │   ROS 2 NODE    │    │
│   │                 │     Port 10000        │                 │    │
│   │  • Robot Model  │◄─────────────────────►│  • ros_tcp_     │    │
│   │  • Sensors      │                       │    endpoint     │    │
│   │  • Environment  │  /joint_states ──────►│                 │    │
│   │                 │  /cmd_vel ◄───────────│  • Controller   │    │
│   │                 │  /camera/image ──────►│  • Planner      │    │
│   │                 │                       │                 │    │
│   └─────────────────┘                       └─────────────────┘    │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### ROS TCP Endpoint (Python)

```python
#!/usr/bin/env python3
"""
ros_tcp_endpoint.py - Bridge between Unity and ROS 2
Install: pip install ros-tcp-endpoint
"""

import rclpy
from rclpy.node import Node
from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist


class UnityROSBridge(Node):
    def __init__(self):
        super().__init__('unity_ros_bridge')

        # TCP Server for Unity connection
        self.tcp_server = TcpServer(self, tcp_ip='0.0.0.0', tcp_port=10000)

        # Register publishers (Unity -> ROS)
        self.tcp_server.source_destination_dict = \{
            'joint_states': RosPublisher(
                self, JointState, 'joint_states', queue_size=10
            ),
        \}

        # Register subscribers (ROS -> Unity)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )

        self.get_logger().info('Unity ROS Bridge started on port 10000')

    def cmd_vel_callback(self, msg):
        # Forward to Unity via TCP
        self.tcp_server.send_unity_message('cmd_vel', msg)


def main():
    rclpy.init()
    node = UnityROSBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Unity Publisher Script

```csharp
// JointStatePublisher.cs - Publish joint states to ROS
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStatePublisher : MonoBehaviour
\{
    private ROSConnection ros;
    private ArticulationBody[] joints;
    private float publishRate = 50f;
    private float lastPublishTime;

    void Start()
    \{
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher&lt;JointStateMsg&gt;("/joint_states");

        // Find all articulation bodies (joints)
        joints = GetComponentsInChildren&lt;ArticulationBody&gt;();
    \}

    void FixedUpdate()
    \{
        if (Time.time - lastPublishTime &gt; 1f / publishRate)
        \{
            PublishJointStates();
            lastPublishTime = Time.time;
        \}
    \}

    void PublishJointStates()
    \{
        var msg = new JointStateMsg();
        msg.header.stamp.sec = (int)Time.time;

        msg.name = new string[joints.Length];
        msg.position = new double[joints.Length];
        msg.velocity = new double[joints.Length];

        for (int i = 0; i &lt; joints.Length; i++)
        \{
            msg.name[i] = joints[i].name;
            msg.position[i] = joints[i].jointPosition[0];
            msg.velocity[i] = joints[i].jointVelocity[0];
        \}

        ros.Publish("/joint_states", msg);
    \}
\}
```

---

## 6.14 Importing Humanoid Robots into Unity

### URDF Import Workflow

1. **Export URDF** from your robot package
2. **Import in Unity** using URDF Importer
3. **Configure joints** as ArticulationBody
4. **Add controllers** for movement

```
URDF File (.urdf)
       │
       ▼
┌──────────────────┐
│  URDF Importer   │
│  (Unity Package) │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│  GameObject      │
│  Hierarchy:      │
│  • base_link     │
│  • joint_1       │
│  • link_1        │
│  • joint_2       │
│  • link_2        │
│  └─ ...          │
└──────────────────┘
```

### FBX Import for Visual Quality

For better visuals, combine URDF structure with FBX meshes:

1. Export meshes from CAD as FBX
2. Import FBX into Unity
3. Replace URDF visual meshes with FBX models
4. Apply PBR materials

### Humanoid Animation

Unity's **Animator** system enables humanoid motion:

```csharp
// HumanoidController.cs - Control humanoid joints
using UnityEngine;

public class HumanoidController : MonoBehaviour
\{
    public ArticulationBody hipJoint;
    public ArticulationBody kneeJoint;
    public ArticulationBody ankleJoint;

    public void SetLegPose(float hip, float knee, float ankle)
    \{
        SetJointTarget(hipJoint, hip);
        SetJointTarget(kneeJoint, knee);
        SetJointTarget(ankleJoint, ankle);
    \}

    private void SetJointTarget(ArticulationBody joint, float angle)
    \{
        var drive = joint.xDrive;
        drive.target = angle;
        joint.xDrive = drive;
    \}

    // Simple walking pattern
    public void WalkCycle(float phase)
    \{
        float hipAngle = Mathf.Sin(phase) * 30f;
        float kneeAngle = Mathf.Cos(phase) * 20f + 20f;
        SetLegPose(hipAngle, kneeAngle, 0f);
    \}
\}
```

---

## 6.15 Human-Robot Interaction Visualization

Unity excels at visualizing human-robot scenarios:

### Use Cases

| Scenario | Unity Features |
|----------|----------------|
| Collaborative assembly | Human animation + robot sync |
| Service robots | Environment with humans |
| Safety visualization | Collision zones, proximity |
| Training simulations | VR/AR integration |

### Safety Zone Visualization

```csharp
// SafetyZoneVisualizer.cs
using UnityEngine;

public class SafetyZoneVisualizer : MonoBehaviour
\{
    public Transform robot;
    public float warningRadius = 2.0f;
    public float dangerRadius = 1.0f;

    private Material zoneMaterial;

    void Update()
    \{
        // Find nearest human
        GameObject[] humans = GameObject.FindGameObjectsWithTag("Human");
        float minDist = float.MaxValue;

        foreach (var human in humans)
        \{
            float dist = Vector3.Distance(robot.position, human.transform.position);
            minDist = Mathf.Min(minDist, dist);
        \}

        // Update zone color
        if (minDist &lt; dangerRadius)
            zoneMaterial.color = Color.red;
        else if (minDist &lt; warningRadius)
            zoneMaterial.color = Color.yellow;
        else
            zoneMaterial.color = Color.green;
    \}
\}
```

---

## 6.16 Digital Twin Mini-Project

**Project Goal**: Create a virtual robotics lab with a humanoid robot, connect to ROS 2, and visualize sensor data.

### Project Components

```
┌─────────────────────────────────────────────────────────────────────┐
│                    MINI-PROJECT ARCHITECTURE                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   UNITY SCENE                          ROS 2 SYSTEM                 │
│   ───────────                          ────────────                 │
│   ┌─────────────────┐                  ┌─────────────────┐         │
│   │  Lab Environment│                  │  ros_tcp_       │         │
│   │  • Floor        │◄────────────────►│  endpoint       │         │
│   │  • Walls        │   TCP Bridge     │                 │         │
│   │  • Lighting     │                  │  Joint State    │         │
│   └─────────────────┘                  │  Publisher      │         │
│   ┌─────────────────┐                  │                 │         │
│   │  Humanoid Robot │                  │  Velocity       │         │
│   │  • 6 DOF Arm    │──────────────────│  Controller     │         │
│   │  • Gripper      │  /joint_states   │                 │         │
│   │  • Camera       │  /cmd_vel        │  Sensor         │         │
│   └─────────────────┘  /camera         │  Processor      │         │
│   ┌─────────────────┐                  └─────────────────┘         │
│   │  Sensors        │                  ┌─────────────────┐         │
│   │  • RGB Camera   │─────────────────►│  RViz2          │         │
│   │  • Depth Camera │  Image Topics    │  Visualization  │         │
│   └─────────────────┘                  └─────────────────┘         │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Step 1: Environment Setup

Create a Unity scene with:
- Floor plane with physics material
- Walls and obstacles
- Lighting (directional + point lights)
- Spawn point for robot

### Step 2: Robot Model

Import humanoid URDF:

```bash
# In Unity, use URDF Importer
# File > Import Robot from URDF
# Select your_robot.urdf
```

Configure ArticulationBody settings:
- Joint limits
- Drive stiffness/damping
- Collision layers

### Step 3: Sensor Simulation

Add camera sensor:

```csharp
// CameraSensor.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraSensor : MonoBehaviour
\{
    public Camera sensorCamera;
    public int width = 640;
    public int height = 480;
    public float publishRate = 30f;

    private ROSConnection ros;
    private RenderTexture renderTexture;
    private Texture2D texture2D;

    void Start()
    \{
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher&lt;ImageMsg&gt;("/camera/image_raw");

        renderTexture = new RenderTexture(width, height, 24);
        texture2D = new Texture2D(width, height, TextureFormat.RGB24, false);
        sensorCamera.targetTexture = renderTexture;

        InvokeRepeating("PublishImage", 0f, 1f / publishRate);
    \}

    void PublishImage()
    \{
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        texture2D.Apply();

        var msg = new ImageMsg();
        msg.width = (uint)width;
        msg.height = (uint)height;
        msg.encoding = "rgb8";
        msg.data = texture2D.GetRawTextureData();

        ros.Publish("/camera/image_raw", msg);
    \}
\}
```

### Step 4: ROS 2 Bridge

Launch ROS TCP endpoint:

```bash
# Terminal 1: Start ROS TCP endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# Terminal 2: Run your control node
ros2 run my_robot joint_controller

# Terminal 3: Visualize in RViz2
rviz2
```

### Step 5: Logging and Debugging

Unity console logging:

```csharp
// DebugLogger.cs
using UnityEngine;

public class DebugLogger : MonoBehaviour
\{
    public ArticulationBody[] joints;

    void Update()
    \{
        if (Input.GetKeyDown(KeyCode.D))
        \{
            foreach (var joint in joints)
            \{
                Debug.Log($"Joint: \{joint.name\}, " +
                         $"Position: \{joint.jointPosition[0]:F3\}, " +
                         $"Velocity: \{joint.jointVelocity[0]:F3\}");
            \}
        \}
    \}
\}
```

ROS 2 bag recording:

```bash
# Record all topics
ros2 bag record -a -o unity_sim_recording

# Play back
ros2 bag play unity_sim_recording
```

---

## 6.17 Chapter Summary

This chapter covered the complete Digital Twin simulation pipeline for Physical AI:

**Part A - Gazebo Simulation:**
- Digital Twin concepts and sim-to-real considerations
- Gazebo architecture, SDF world files, physics configuration
- Sensor simulation with realistic noise models
- URDF to SDF conversion workflow
- ros2_control integration for standardized control

**Part B - Unity Visualization:**
- Unity Robotics Hub and high-fidelity rendering
- ROS-Unity integration via TCP Connector
- Humanoid robot import and animation
- Human-robot interaction visualization
- Complete mini-project implementation

### Key Takeaways

1. **Use the right tool**: Gazebo for physics, Unity for visuals
2. **Bridge both worlds**: ROS 2 connects simulation to reality
3. **Model noise**: Realistic sensors improve sim-to-real transfer
4. **Validate continuously**: Digital Twins require ongoing calibration

### Next Chapter Preview

**Chapter 7: NVIDIA Isaac Sim** will explore advanced simulation capabilities:
- GPU-accelerated physics with PhysX 5
- Domain randomization for ML training
- Synthetic data generation pipeline
- Isaac SDK integration for perception
- Sim-to-real transfer techniques

---

## 6.18 Glossary

| Term | Definition |
|------|------------|
| **ArticulationBody** | Unity component for physics-based robot joints with reduced coordinates |
| **Collision Mesh** | Simplified geometry used for physics collision detection |
| **Contact Dynamics** | Physics simulation of surfaces touching and interacting |
| **DART** | Dynamic Animation and Robotics Toolkit — physics engine used by Gazebo |
| **Digital Twin** | Synchronized virtual replica of physical system with bidirectional data flow |
| **Domain Randomization** | Varying simulation parameters (mass, friction, textures) for robust ML training |
| **FBX** | Autodesk's 3D model format supporting meshes, materials, and animation |
| **Friction Coefficient** | Parameter (μ) determining resistance to sliding between surfaces |
| **Gazebo** | Open-source robotics simulator integrated with ROS |
| **GPU Rendering** | Using graphics processing unit for real-time visualization |
| **HDRP** | High Definition Render Pipeline — Unity's photorealistic rendering system |
| **Inertia Tensor** | 3×3 matrix describing mass distribution and rotational resistance |
| **Joint Drive** | Controller that applies forces/torques to achieve target joint states |
| **ODE** | Open Dynamics Engine — physics engine option in Gazebo |
| **PBR Materials** | Physically Based Rendering materials for realistic appearance |
| **PhysX** | NVIDIA physics engine used by Unity and Isaac Sim |
| **Plugin** | Extensible module that adds functionality to Gazebo or Unity |
| **Real-Time Factor** | Ratio of simulation time to wall-clock time (1.0 = real-time) |
| **Restitution** | Coefficient determining bounciness of collisions (0-1) |
| **ROS TCP Connector** | Unity package enabling ROS 2 communication via TCP/IP |
| **SDF** | Simulation Description Format — XML format for Gazebo worlds and models |
| **Sim-to-Real Gap** | Difference between simulated and real-world behavior |
| **Spawn Entity** | ROS 2 service/node for adding models to Gazebo simulation |
| **Timestep** | Duration of each physics simulation step (smaller = more accurate) |
| **URDF** | Unified Robot Description Format — XML format for robot models |
| **URDF Importer** | Unity package for importing robot models from URDF files |
| **URP** | Universal Render Pipeline — Unity's cross-platform rendering system |
| **Visual Mesh** | Detailed geometry used for rendering (separate from collision) |
| **World File** | SDF file defining the complete simulation environment |

---

## 6.19 Review Questions and Exercises

### Conceptual Questions

1. What distinguishes a Digital Twin from a traditional simulation?

2. When would you choose Gazebo over Unity, and vice versa?

3. How does sensor noise modeling improve sim-to-real transfer?

4. Explain the purpose of the ROS TCP Connector in Unity.

5. What are the advantages of ArticulationBody over standard Rigidbody for robots?

6. What is the sim-to-real gap and what factors contribute to it?

7. Explain the difference between SDF and URDF formats. When would you use each?

8. Why is physics timestep important for humanoid simulation? What happens if it's too large?

9. What is domain randomization and how does it help with real-world deployment?

10. Compare the physics engines available in Gazebo (ODE, DART, Bullet). What are the tradeoffs?

### Technical Questions

11. A robot simulation runs at 1000 Hz physics update rate with `max_step_size=0.001`. What is the real-time factor if each step takes 0.8ms to compute?

12. Calculate the inertia tensor for a box with dimensions 0.2m × 0.1m × 0.3m and mass 2.0kg.

13. Your LiDAR sensor in Gazebo publishes 360 rays at 10 Hz with Gaussian noise (σ=0.02m). How many noisy measurements per second?

14. Design a friction model for a humanoid foot. What μ (friction coefficient) values would you use for rubber on concrete?

15. A Unity camera publishes 640×480 RGB images at 30 FPS over ROS TCP. Calculate the bandwidth required in MB/s.

16. Your Gazebo simulation runs at real-time factor 0.7. A 10-second trajectory in simulation takes how long in wall-clock time?

17. Configure a depth camera sensor in SDF with range 0.3-10m, resolution 640×480, and 15 FPS update rate.

18. What solver iterations would you recommend for a 32-DOF humanoid with many ground contacts?

19. Calculate the expected IMU accelerometer reading when a robot is stationary on a 5° slope.

20. Design a collision mesh simplification strategy for a complex humanoid hand with 20 finger links.

### Practical Exercises

**Exercise 1: Gazebo World Creation**
Create a Gazebo world with:
- A ground plane with friction
- Three obstacles (box, cylinder, sphere)
- Directional lighting
- Save as `my_world.sdf`

**Exercise 2: Sensor Configuration**
Add to your robot URDF:
- A LiDAR sensor with 0.02m noise
- An IMU at the base link
- Test in Gazebo and verify data on ROS topics

**Exercise 3: Unity-ROS Integration**
Set up a Unity project that:
- Imports a simple 2-DOF robot arm
- Publishes joint states to ROS 2
- Subscribes to position commands
- Verify bidirectional communication

**Exercise 4: Complete Digital Twin**
Build a complete pipeline:
- Robot in Gazebo with physics
- Visualization in Unity with better graphics
- Both connected to same ROS 2 control node
- Compare behavior in both simulators

**Exercise 5: Mini-Project Extension**
Extend the chapter mini-project:
- Add a second camera (stereo vision)
- Implement basic obstacle detection
- Visualize detected obstacles in Unity
- Log detection events to ROS 2 bag
