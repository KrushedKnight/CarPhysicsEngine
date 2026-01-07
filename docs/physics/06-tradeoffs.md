# Design Tradeoffs and Decisions

## Overview

Building a vehicle physics simulation requires balancing realism, performance, and playability. This document explains the key decisions made during development and the tradeoffs involved.

## Integration Method

### Decision: Semi-Implicit Euler

**Options Considered:**

| Method | Accuracy | Stability | Performance | Complexity |
|--------|----------|-----------|-------------|------------|
| Explicit Euler | Low | Poor | Excellent | Trivial |
| **Semi-Implicit Euler** | Good | Good | Excellent | Simple |
| Velocity Verlet | Good | Excellent | Good | Moderate |
| RK4 | Excellent | Excellent | Poor | High |

**Choice Rationale:**

Semi-implicit Euler was chosen because:
1. **Stability**: Unlike explicit Euler, it doesn't gain energy over time
2. **Simplicity**: Easy to implement and debug
3. **Performance**: Single force evaluation per frame
4. **Sufficient accuracy**: At 60 FPS, timestep is small enough for good results

**Tradeoff accepted**: Slight energy loss over time (damping effect), which actually helps prevent runaway oscillations.

## Tire Model

### Decision: Simplified Pacejka-Inspired Model

**Options Considered:**

| Model | Realism | Tuning Difficulty | Performance |
|-------|---------|-------------------|-------------|
| Linear | Poor | Easy | Excellent |
| Brush Model | Good | Moderate | Good |
| **Sine-Based** | Good | Easy | Excellent |
| Full Pacejka | Excellent | Very Hard | Good |

**Choice Rationale:**

The sine-based lateral force curve:
```cpp
forceMagnitude = maxFrictionForce * sin(normalizedAngle * PI / 2.0);
```

Was chosen because:
1. **Intuitive behavior**: Smooth progression to peak grip
2. **Easy tuning**: Only need peak angle and slide ratio
3. **Computationally cheap**: No lookup tables or complex math
4. **Sufficient realism**: Captures the essential nonlinear grip behavior

**Tradeoff accepted**: Less accurate at extreme slip angles, but these are edge cases in normal driving.

### Decision: Combined Slip via Friction Circle

Rather than implementing complex combined slip formulas, we use simple vector limiting:

```cpp
if (combinedMagnitude > maxFrictionForce) {
    scale = maxFrictionForce / combinedMagnitude;
    // Scale both forces equally
}
```

**Tradeoff accepted**: Real tires have asymmetric combined slip behavior. Our circular model is a reasonable approximation for gameplay.

## Drivetrain

### Decision: First-Principles Engine Model

**Options Considered:**

| Approach | Realism | Flexibility | Development Time |
|----------|---------|-------------|------------------|
| **Thermodynamic** | High | High | Long |
| Torque Curve Lookup | Medium | Low | Short |
| Simple Power Model | Low | Very Low | Minimal |

**Choice Rationale:**

The thermodynamic model calculates power from:
1. Air flow based on volumetric efficiency
2. Fuel combustion with realistic energy values
3. RPM-dependent efficiency curves

This approach:
- Creates natural torque curves without manual tuning
- Responds realistically to altitude/temperature (if implemented)
- Educational value in understanding engine physics

**Tradeoff accepted**: More complex to implement and debug than lookup tables.

### Decision: Slip-Based Clutch Model

Rather than a simple on/off clutch:

```cpp
double slip = engineOmega - transmissionOmega;
torque = slip * CLUTCH_SLIP_K;
```

**Benefits:**
- Smooth engagement without jerk
- Realistic stall behavior
- Natural feel during shifts

**Tradeoff accepted**: More complex state management than binary clutch.

## Steering

### Decision: Full Ackermann Geometry

**Options Considered:**

| Approach | Realism | Complexity |
|----------|---------|------------|
| Parallel steering | Poor | Trivial |
| **Full Ackermann** | Excellent | Moderate |
| Variable Ackermann | Excellent | High |

**Choice Rationale:**

Computing individual wheel angles:
```cpp
innerAngle = atan(wheelbase / innerRadius);
outerAngle = atan(wheelbase / outerRadius);
```

Prevents tire scrub and improves handling feel significantly with minimal computational cost.

**Tradeoff accepted**: More complex steering code, but essential for believable physics.

## Weight Transfer

### Decision: Quasi-Static Model

We use instantaneous weight transfer based on current acceleration:

```cpp
dFz = -mass * acceleration * cg_height / wheelbase;
```

**Not implemented**: Dynamic weight transfer (suspension compression/extension over time).

**Rationale:**
- Suspension simulation adds significant complexity
- Quasi-static model captures the essential behavior
- Good enough for the 2D top-down perspective

**Tradeoff accepted**: Less realistic transient behavior, but simpler and stable.

## Control Systems

### Decision: PD Control for TCS/ABS

**Why not PID?**

The integral term would:
1. Accumulate error during normal driving
2. Cause unwanted intervention when grip returns
3. Make tuning more difficult

**Why not more sophisticated control?**

| Method | Response | Complexity | Tuning |
|--------|----------|------------|--------|
| **PD** | Fast | Low | Easy |
| PID | Faster | Medium | Medium |
| Model Predictive | Optimal | Very High | Expert |
| Fuzzy Logic | Smooth | High | Medium |

PD provides adequate performance with simple tuning.

## Coordinate Systems

### Decision: Separate Local/World Calculations

Forces are computed in wheel-local coordinates, then transformed to world:

```cpp
// Local calculation
Vector2d forceLocal = wheel.calculateFriction(velocityLocal, dt);

// Transform to world
Vector2d forceWorld = rotateToWorld(forceLocal, carAngle);
```

**Benefits:**
- Tire physics independent of car orientation
- Easier to reason about slip angles
- Cleaner separation of concerns

**Tradeoff accepted**: Extra transformation overhead (minimal).

## Fixed vs Variable Timestep

### Decision: Fixed 16ms Timestep

**Benefits:**
- Deterministic physics
- Consistent behavior across hardware
- Simpler collision detection
- No numerical instability from large dt

**Tradeoffs accepted:**
- May not perfectly match display refresh rate
- Slight input latency on high-refresh displays

## Parameters and Tuning

### Decision: Centralized Constants

All physics parameters in header files:

```cpp
namespace PhysicsConstants {
    constexpr double WHEEL_RADIUS = 0.33;
    constexpr double CLUTCH_MAX_TORQUE = 750.0;
    // ...
}
```

**Benefits:**
- Easy to adjust without code changes
- Clear documentation of magic numbers
- Consistent values across modules

**Tradeoff**: Recompilation required for changes (acceptable for game release).

## What Was Left Out

Features deliberately not implemented:

| Feature | Reason |
|---------|--------|
| Suspension | Complexity, 2D view doesn't benefit |
| Aerodynamics | Minimal effect at game speeds |
| Tire temperature | Complexity, short race duration |
| Engine temperature | Complexity, no failure simulation |
| Differential | RWD works well with open diff assumption |
| Fuel consumption | Adds little to gameplay |

## Lessons Learned

1. **Start simple**: The initial tire model was linear. Complexity was added incrementally.

2. **Validate often**: Console logging of key values (slip, forces, torques) caught many bugs early.

3. **Tune iteratively**: Initial parameters rarely felt right. Many adjustments were made based on driving feel.

4. **Keep physics and rendering separate**: Clean boundaries made debugging much easier.

5. **Test edge cases**: Very low speed, neutral gear, and maximum slip all required special handling.

---

**Previous: [Driver Assistance Systems](05-driver-aids.md)** | **Back to: [Overview](00-overview.md)**
