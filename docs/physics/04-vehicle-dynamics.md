# Vehicle Dynamics

## Overview

The `Car` class integrates all physics subsystems into a complete vehicle simulation. It manages four wheels, steering geometry, weight transfer, and the transformation between local and world coordinates.

## Wheel Configuration

The vehicle uses a four-wheel layout with rear-wheel drive:

```
      Front
   FL ──┬── FR    (steered)
        │
        │ Wheelbase
        │
   RL ──┴── RR    (driven)
      Rear
```

### Wheel Positioning

Wheels are positioned relative to the car's center:

```cpp
double halfWidth = (CAR_WIDTH / 10.0) / 2.0;
double halfLength = (CAR_LENGTH / 10.0) / 2.0;

frontLeft->position = Vector2d(-halfWidth + WHEEL_WIDTH_INSET,
                                halfLength - WHEEL_LENGTH_INSET);
```

The insets (`WHEEL_WIDTH_INSET = 0.1m`, `WHEEL_LENGTH_INSET = 0.15m`) place wheels inside the body for visual realism.

## Steering System

### Ackermann Geometry

Real cars use Ackermann steering where inner and outer wheels turn at different angles:

```
                Turn Center
                    ●
                   /│\
                  / │ \
                 /  │  \
    Inner Wheel/   │   \Outer Wheel
         ●────────●────────●
              Wheelbase
```

The inner wheel must turn more sharply because it traces a smaller circle:

```cpp
double turnRadius = wheelbase / tan(abs(baseAngle));

if (steering_angle > 0) {
    double innerRadius = turnRadius - trackWidth / 2.0;
    double outerRadius = turnRadius + trackWidth / 2.0;
    frontLeft->wheelAngle = atan(wheelbase / innerRadius);   // More angle
    frontRight->wheelAngle = atan(wheelbase / outerRadius);  // Less angle
}
```

This prevents tire scrub during turns and improves handling accuracy.

### Speed-Sensitive Steering

Steering ratio reduces at high speed for stability:

```cpp
double speed = velocity.norm();
double maxSpeed = 50.0;
double speedFactor = 1.0 - (speed / maxSpeed) * 0.5;
speedFactor = max(0.5, speedFactor);
steering_angle += amount * speedFactor;
```

At maximum speed, steering input is halved, preventing twitchy behavior.

### Force Feedback (Self-Centering)

Steering naturally returns to center:

```cpp
steering_angle *= FORCE_FEEDBACK_DECAY;  // 0.985
```

This 1.5% decay per frame simulates caster trail and tire self-aligning torque.

## Coordinate Transformations

### Wheel Velocity Calculation

Each wheel has a unique velocity due to car rotation:

```cpp
Vector2d calculateWheelVelocityLocal(Vector2d wheelPosition) {
    // Transform car velocity to local coordinates
    Vector2d velocityLocal(
        velocity.x() * cos(angle) - velocity.y() * sin(angle),
        velocity.x() * sin(angle) + velocity.y() * cos(angle)
    );

    // Add rotational component
    Vector2d rotationalVelLocal(
        -angular_velocity * wheelPosition.y(),
        angular_velocity * wheelPosition.x()
    );

    return velocityLocal + rotationalVelLocal;
}
```

During rotation:
- Front-left wheel moves right relative to car
- Front-right wheel moves left relative to car
- This creates natural understeer/oversteer behavior

### Force Transformation

Tire forces computed in wheel-local coordinates must be transformed to world coordinates:

```cpp
Vector2d wheelForceWorld(
    wheelForceLocal.x() * cos(angle) + wheelForceLocal.y() * sin(angle),
    -wheelForceLocal.x() * sin(angle) + wheelForceLocal.y() * cos(angle)
);
```

## Weight Transfer

### Load Transfer Model

Acceleration shifts weight between axles and sides:

```cpp
// Longitudinal transfer (braking/acceleration)
double dFz_longitudinal = -mass * ay_local * cg_height / wheelbase;

// Lateral transfer (cornering)
double dFz_lateral = -mass * ax_local * cg_height / track_width;
```

Where:
- `cg_height = 0.5m` (center of gravity height)
- `ay_local` = forward/backward acceleration
- `ax_local` = lateral acceleration

### Weight Distribution

```cpp
double frontWeightBias = 0.6;  // 60% front
double rearWeightBias = 0.4;   // 40% rear

frontLeft->normalForce = frontNominalLoad + dFz_longitudinal - dFz_lateral;
frontRight->normalForce = frontNominalLoad + dFz_longitudinal + dFz_lateral;
backLeft->normalForce = rearNominalLoad - dFz_longitudinal - dFz_lateral;
backRight->normalForce = rearNominalLoad - dFz_longitudinal + dFz_lateral;
```

**Effects:**
- Hard braking: Weight shifts forward, rear wheels lose grip
- Hard acceleration: Weight shifts backward, front wheels lose grip
- Left turn: Weight shifts right, left wheels lose grip

### Minimum Load Clamping

Prevents negative loads (wheel lift):

```cpp
wheel->normalForce = max(60.0, wheel->normalForce);
```

This 60N minimum ensures wheels always have some grip.

## Torque Generation

### Yaw Moment Calculation

Each wheel's force creates torque about the car's center:

```cpp
double torque = wheelPosition.x() * wheelForceLocal.y()
              - wheelPosition.y() * wheelForceLocal.x();
```

This cross product determines:
- Front wheels steering left → positive torque (counter-clockwise)
- Rear wheels sliding right → positive torque
- Asymmetric forces → rotation

### Force Summation

All wheel forces are accumulated on the chassis:

```cpp
for (Wheel* wheel : wheels) {
    Vector2d wheelForceWorld = transformToWorld(wheel->calculateFriction(...));
    addForce(wheelForceWorld, wheelName);
    addTorque(torque);
}
```

Named forces enable the debug free-body diagram visualization.

## Input Smoothing

Raw inputs are smoothed to simulate pedal/wheel travel time:

```cpp
const double throttleRate = 6.0;  // Full travel in ~167ms
const double brakeRate = 9.0;     // Full travel in ~111ms
const double steeringRate = 7.0;  // Full travel in ~143ms

double throttleDiff = targetThrottle - actualThrottle;
double throttleChange = clamp(throttleDiff, -throttleRate * dt, throttleRate * dt);
actualThrottle += throttleChange;
```

This prevents instant input changes that would cause unrealistic behavior.

## Vehicle Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `CAR_MASS` | 1200 kg | Total vehicle mass |
| `CAR_WEIGHT` | 11772 N | Mass × gravity |
| `CAR_TOP_SPEED` | 50 m/s | ~180 km/h |
| `MAX_STEERING_ANGLE` | 55° | Maximum steering lock |
| `STEERING_RACK` | 1.5 | Steering ratio |
| `CG_HEIGHT` | 0.5 m | Center of gravity height |
| `BRAKING_POWER` | 8000 | Brake torque constant |

## Update Sequence

Each frame, the car updates in this order:

```cpp
// 1. Process inputs with smoothing
updateInputs(TIME_INTERVAL);

// 2. Update drivetrain
updateEngine(throttle);

// 3. Apply brakes through ABS
applyBrakes();

// 4. Update wheel rotation
moveWheels();

// 5. Calculate and apply tire forces
sumWheelForces();

// 6. Integrate rigid body motion
incrementTime(TIME_INTERVAL);
```

This ordering ensures forces are calculated with current state before integration.

---

**Previous: [Drivetrain System](03-drivetrain.md)** | **Next: [Driver Assistance Systems](05-driver-aids.md)**
