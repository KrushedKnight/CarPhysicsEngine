# BasicCarGame Physics Documentation

A comprehensive guide to the vehicle physics simulation powering BasicCarGame.

## Table of Contents

1. **[Overview & Architecture](00-overview.md)**
   - System architecture and module organization
   - Physics pipeline and update sequence
   - Coordinate systems and units

2. **[Rigid Body Physics](01-rigid-body.md)**
   - State variables and integration
   - Semi-implicit Euler method
   - Force accumulation and transformation

3. **[Tire Physics Model](02-tire-model.md)**
   - Slip ratio and slip angle concepts
   - Lateral and longitudinal force generation
   - Load sensitivity and friction circle
   - Low-speed handling

4. **[Drivetrain System](03-drivetrain.md)**
   - Thermodynamic engine simulation
   - Gearbox and gear ratios
   - Clutch engagement physics
   - Reflected inertia calculations

5. **[Vehicle Dynamics](04-vehicle-dynamics.md)**
   - Four-wheel configuration
   - Ackermann steering geometry
   - Weight transfer model
   - Coordinate transformations

6. **[Driver Assistance Systems](05-driver-aids.md)**
   - Traction Control System (TCS)
   - Anti-lock Braking System (ABS)
   - PD control theory and implementation

7. **[Design Tradeoffs](06-tradeoffs.md)**
   - Integration method selection
   - Tire model simplifications
   - Feature prioritization
   - Lessons learned

## Quick Reference

### Key Parameters

| Parameter | Value | Unit |
|-----------|-------|------|
| Vehicle Mass | 1200 | kg |
| Wheel Radius | 0.33 | m |
| Max RPM | 8000 | rpm |
| Timestep | 16 | ms |
| Gear Count | 6 + R | - |

### Architecture Diagram

```
┌─────────────────────────────────────────────────────────┐
│                         Car                              │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐    │
│  │   FL    │  │   FR    │  │   RL    │  │   RR    │    │
│  │  Wheel  │  │  Wheel  │  │  Wheel  │  │  Wheel  │    │
│  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘    │
│       │            │            │            │          │
│       └────────────┴─────┬──────┴────────────┘          │
│                          │                              │
│  ┌─────────┐      ┌──────┴──────┐      ┌─────────┐     │
│  │  Engine │──────│   Gearbox   │──────│   TCS   │     │
│  └─────────┘      └─────────────┘      └─────────┘     │
│                                                         │
│  ┌─────────┐      ┌─────────────┐                      │
│  │   ABS   │──────│  RigidBody  │                      │
│  └─────────┘      └─────────────┘                      │
└─────────────────────────────────────────────────────────┘
```

## Technologies Used

- **C++17** - Core implementation
- **Eigen** - Linear algebra (Vector2d)
- **SDL2** - Rendering and input

## Building

The physics system has no external dependencies beyond Eigen (header-only). It compiles as part of the main game build:

```bash
mkdir build && cd build
cmake ..
make
```

## Testing

Physics tests use GoogleTest:

```bash
./build/tests/RunAllTests
```

Test coverage includes:
- Rigid body integration
- Wheel slip calculations
- Engine torque curves
- Gearbox state transitions
