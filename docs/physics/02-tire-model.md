# Tire Physics Model

## Overview

The tire model is the most critical component of the vehicle simulation. Tires are the only connection between the vehicle and the ground, and all acceleration, braking, and cornering forces must pass through them. This implementation uses a simplified model inspired by the Pacejka "Magic Formula" but optimized for real-time performance.

## Fundamental Concepts

### Slip Ratio (Longitudinal)

Slip ratio describes the difference between wheel rotational speed and vehicle speed:

```
slip_ratio = (wheel_speed - vehicle_speed) / |vehicle_speed|
```

Where:
- `wheel_speed = angular_velocity × radius`
- `vehicle_speed = velocity component along wheel direction`

| Slip Ratio | Meaning |
|------------|---------|
| 0 | Perfect rolling (no slip) |
| > 0 | Wheel spinning faster (acceleration) |
| < 0 | Wheel spinning slower (braking) |
| 1.0 | Full wheelspin (100% slip) |
| -1.0 | Locked wheel (100% braking slip) |

### Slip Angle (Lateral)

Slip angle is the angle between the wheel's pointing direction and its actual travel direction:

```cpp
double slipAngle = std::atan2(std::abs(lateralVelocity), std::abs(forwardVelocity));
```

This angle generates the cornering force that allows the vehicle to turn.

## Force Generation

### Lateral Force Model

The lateral force uses a sine-based curve that approximates real tire behavior:

```
            Force
              │
    maxForce ─┼─────────╮
              │        ╱ ╲______ (exponential decay)
              │      ╱
              │    ╱
              │  ╱
              │╱
              └──────────────────── Slip Angle
                   peak    slide
                  (8°)    region
```

**Below Peak Slip Angle (8°):**
```cpp
forceMagnitude = maxFrictionForce * sin(normalizedAngle * PI / 2.0);
```

This creates a smooth, progressive force buildup that peaks at the optimal slip angle.

**Above Peak Slip Angle:**
```cpp
double excessAngle = slipAngle - TIRE_PEAK_SLIP_ANGLE;
double decayRate = 8.0;
forceMagnitude = maxFrictionForce * (slideRatio + (1.0 - slideRatio) * exp(-decayRate * excessAngle));
```

The exponential decay simulates the tire transitioning from grip to slide, where it maintains approximately 75% of peak grip (`TIRE_SLIDE_RATIO = 0.75`).

### Longitudinal Force Model

Longitudinal forces use a velocity-matching approach:

```cpp
double requiredForce = (longitudinalSlip / time_interval) * wheelMass * FRICTION_RESPONSE;
longitudinalFriction = clamp(requiredForce, -maxFrictionForce, maxFrictionForce);
```

This calculates the force needed to eliminate the slip, then clamps it to the available friction.

### Low-Speed Handling

At speeds below 0.5 m/s, the tire model switches to a simpler velocity-elimination approach:

```cpp
if (speed < TIRE_LOW_SPEED_THRESHOLD) {
    double requiredLateralForce = -(lateralVelocity / time_interval) * wheelMass * LATERAL_FRICTION_RESPONSE;
    lateralFriction = clamp(requiredLateralForce, -maxFrictionForce, maxFrictionForce);
}
```

This prevents numerical instability and unrealistic behavior at parking speeds where slip angles become undefined.

## Load Sensitivity

Real tires don't scale linearly with load—doubling the weight doesn't double the grip. This is modeled using:

```cpp
double nominalLoad = 2943.0;  // N (300 kg per wheel)
double loadSensitivity = 0.9;
double loadFactor = pow(normalForce / nominalLoad, loadSensitivity);
double maxFrictionForce = nominalLoad * frictionCoefficient * loadFactor;
```

The exponent of 0.9 creates a diminishing returns curve:

| Load (N) | Linear Grip (N) | Actual Grip (N) | Efficiency |
|----------|-----------------|-----------------|------------|
| 1500 | 1500 | 1580 | 105% |
| 2943 | 2943 | 2943 | 100% |
| 4000 | 4000 | 3780 | 94% |
| 6000 | 6000 | 5340 | 89% |

This models how heavily-loaded tires (outer wheels in corners) have reduced efficiency.

## Friction Circle

The combined longitudinal and lateral forces are limited by the friction circle (friction ellipse):

```cpp
double combinedMagnitude = sqrt(longitudinalFriction² + lateralFriction²);

if (combinedMagnitude > maxFrictionForce) {
    double scale = maxFrictionForce / combinedMagnitude;
    longitudinalFriction *= scale;
    lateralFriction *= scale;
}
```

This ensures that:
- Full braking = no cornering ability
- Full cornering = reduced acceleration
- Combined maneuvers share the available grip

```
        Lateral
           │
           │    ╱ Friction Circle
      ─────┼───●─────
           │    ╲
           │
    ───────┼─────────── Longitudinal
           │
```

## Wheel Rotation

The wheel's angular velocity is updated based on applied torques:

```cpp
void Wheel::incrementTime(double time_interval) {
    angular_acceleration = angular_torque / moment_of_inertia;
    angular_position += angular_velocity * time_interval + 0.5 * angular_acceleration * time_interval²;
    angular_velocity += angular_acceleration * time_interval;
    clearTorques();
}
```

Friction torque from ground contact opposes wheel spin:

```cpp
double wheelEffectiveMass = moment_of_inertia / (wheelRadius * wheelRadius);
double frictionTorque = longitudinalFriction * wheelRadius * (wheelEffectiveMass / wheelMass);
addTorque(-frictionTorque);
```

## Key Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `WHEEL_RADIUS` | 0.33 m | Standard tire radius |
| `WHEEL_MASS` | 20 kg | Unsprung mass per wheel |
| `WHEEL_FRICTION` | 1.0 | Coefficient of friction |
| `TIRE_PEAK_SLIP_ANGLE` | 8° | Optimal cornering angle |
| `TIRE_SLIDE_RATIO` | 0.75 | Grip retained when sliding |
| `TIRE_LOW_SPEED_THRESHOLD` | 0.5 m/s | Switch to low-speed model |

## Grip Level Tracking

Each wheel reports its current grip utilization:

```cpp
gripLevel = (maxFrictionForce > 0.0) ? (combinedMagnitude / maxFrictionForce) : 0.0;
```

This value (0.0 to 1.0) is displayed in the telemetry UI, showing how close each tire is to the limit.

---

**Previous: [Rigid Body Physics](01-rigid-body.md)** | **Next: [Drivetrain System](03-drivetrain.md)**
