# Rigid Body Physics

## Overview

The `RigidBody` class serves as the foundation for all physics objects in the simulation. It implements 2D rigid body dynamics with both linear and rotational motion, using Newton's laws of motion.

## State Variables

Each rigid body maintains the following state:

| Variable | Type | Description |
|----------|------|-------------|
| `pos_x`, `pos_y` | double | Position in pixels |
| `velocity` | Vector2d | Linear velocity in m/s |
| `acceleration` | Vector2d | Linear acceleration in m/s² |
| `forces` | Vector2d | Accumulated force vector in N |
| `angular_position` | double | Rotation angle in radians |
| `angular_velocity` | double | Angular velocity in rad/s |
| `angular_acceleration` | double | Angular acceleration in rad/s² |
| `angular_torque` | double | Accumulated torque in N·m |
| `mass` | double | Mass in kg |
| `moment_of_inertia` | double | Rotational inertia in kg·m² |

## Integration Method

The simulation uses **semi-implicit Euler integration**, which provides a good balance between accuracy and computational efficiency for real-time applications.

### Position Update

Position is calculated using the kinematic equation:

```
Δx = v·Δt + ½·a·Δt²
```

In code:
```cpp
double dx_meters = velocity.x() * time_interval +
                   0.5 * acceleration.x() * time_interval * time_interval;
```

This second-order term improves accuracy during acceleration, providing better energy conservation than simple Euler integration.

### Velocity Update

Velocity is updated after position:

```
v_new = v_old + a·Δt
```

### Angular Motion

Rotational dynamics follow the same pattern:

```
θ_new = θ_old + ω·Δt + ½·α·Δt²
ω_new = ω_old + α·Δt
```

Angular position is normalized to [-π, π] using `std::remainder` to prevent floating-point precision loss at large angles.

## Force Accumulation

Forces are accumulated each frame before integration:

```cpp
void RigidBody::addForce(Eigen::Vector2d force, const std::string& name) {
    forces += force;
    if (!name.empty()) {
        namedForces[name] += force;
    }
}
```

Named forces enable debug visualization, showing individual force contributions (tire friction, drag, etc.) in the free body diagram overlay.

## Acceleration Calculation

Newton's second law is applied directly:

```cpp
acceleration = forces / mass;
angular_acceleration = angular_torque / moment_of_inertia;
```

## Design Considerations

### Why Semi-Implicit Euler?

| Method | Pros | Cons |
|--------|------|------|
| Explicit Euler | Simple | Energy gain, unstable at high speeds |
| **Semi-Implicit Euler** | Stable, simple, fast | Slight energy damping |
| Verlet | Energy conserving | Complex force handling |
| RK4 | Highly accurate | 4x force evaluations |

For a real-time game at 60 FPS, semi-implicit Euler provides sufficient accuracy without the computational overhead of higher-order methods.

### Coordinate Systems

The simulation maintains separate coordinate systems:
- **World coordinates**: Absolute position in meters/pixels
- **Local coordinates**: Relative to object's orientation

Transformation between systems uses standard rotation matrices:

```cpp
// World to local
velocityLocal.x() = velocity.x() * cos(angle) - velocity.y() * sin(angle);
velocityLocal.y() = velocity.x() * sin(angle) + velocity.y() * cos(angle);
```

### Force Clearing

Forces are cleared after each integration step:

```cpp
clearForces();
clearTorques();
```

This ensures forces are applied only for the frame in which they're generated, preventing accumulation bugs.

## Car-Specific Parameters

For the vehicle body:

| Parameter | Value | Source |
|-----------|-------|--------|
| Mass | 1200 kg | `PhysicsConstants::CAR_MASS` |
| Moment of Inertia | Calculated | Based on car dimensions |

The moment of inertia is computed at runtime based on screen dimensions to maintain consistent physics across different display sizes.

---

**Previous: [Overview](00-overview.md)** | **Next: [Tire Physics](02-tire-model.md)**
