# BasicCarGame Physics System Overview

## Introduction

BasicCarGame is a 2D vehicle physics simulation that models realistic car dynamics including tire physics, drivetrain mechanics, and driver assistance systems. The simulation runs at 60 FPS with a fixed timestep of 16ms, providing consistent and deterministic physics behavior.

## Architecture

The physics system is organized into six logical modules:

```
src/
├── core/           # Fundamental physics primitives
│   └── RigidBody   # 2D rigid body with forces and torques
├── vehicle/        # Vehicle components
│   ├── Car         # Complete vehicle dynamics
│   ├── Wheel       # Tire physics and friction
│   ├── Engine      # Internal combustion simulation
│   └── Gearbox     # Manual transmission with clutch
├── control/        # Driver assistance systems
│   ├── TractionControl
│   └── AntiLockBrakes
├── config/         # Physics constants and parameters
└── rendering/      # Visualization (Camera, Ground, etc.)
```

## Physics Pipeline

Each simulation frame executes the following sequence:

1. **Input Processing** - Throttle, brake, and steering inputs are smoothed
2. **Engine Update** - Torque calculated from thermodynamics
3. **Gearbox Update** - Clutch engagement and torque transmission
4. **Wheel Torques** - Drive torque applied through TCS
5. **Brake Torques** - Brake torque applied through ABS
6. **Load Transfer** - Weight distribution updated from acceleration
7. **Tire Forces** - Friction calculated for each wheel
8. **Force Summation** - All forces/torques accumulated on chassis
9. **Integration** - Position and velocity updated via Euler integration

## Key Features

### Realistic Tire Model
- Slip angle-based lateral force generation
- Slip ratio-based longitudinal forces
- Combined slip friction circle limiting
- Load sensitivity with non-linear grip scaling

### Thermodynamic Engine
- Volumetric efficiency curves
- Air-fuel combustion modeling
- RPM-based power delivery
- Friction and load torque simulation

### Manual Transmission
- Multi-speed gearbox with final drive
- Clutch engagement simulation with bite point
- Slip-based torque transmission
- Reflected inertia calculations

### Driver Aids
- PD-controlled traction control system
- PD-controlled anti-lock braking
- Real-time slip ratio monitoring

## Coordinate System

The simulation uses a right-handed coordinate system:
- **X-axis**: Positive right
- **Y-axis**: Positive forward (up on screen)
- **Angular position**: Positive counter-clockwise

Local wheel coordinates are transformed to world coordinates using rotation matrices, maintaining consistent force application regardless of vehicle orientation.

## Units

All physics calculations use SI units:
- Distance: meters
- Mass: kilograms
- Time: seconds
- Force: Newtons
- Torque: Newton-meters
- Angle: radians

A scaling factor (25 pixels per meter) converts between physics space and screen space.

---

**Next: [Rigid Body Physics](01-rigid-body.md)**
