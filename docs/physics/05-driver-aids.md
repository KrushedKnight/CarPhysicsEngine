# Driver Assistance Systems

## Overview

The simulation implements two active safety systems found in modern vehicles: Traction Control System (TCS) and Anti-lock Braking System (ABS). Both use PD (Proportional-Derivative) control to regulate wheel slip.

## Control Theory Background

### PD Controller

Both systems use the same control structure:

```
output = Kp × error + Kd × derivative
```

Where:
- `error` = setpoint - measured_value
- `derivative` = change in measured value
- `Kp` = proportional gain (immediate response)
- `Kd` = derivative gain (damping/prediction)

**Why PD instead of PID?**

The integral term in PID would accumulate error during normal driving (when slip is acceptable), causing unwanted corrections. PD provides fast response without this issue.

## Traction Control System (TCS)

### Purpose

TCS prevents wheelspin during acceleration by modulating engine torque when driven wheels slip excessively.

### Slip Ratio Monitoring

```cpp
double slipRatio = wheel.calculateSlipRatio(wheelVelocityLocal);
```

| Slip Ratio | Condition | TCS Action |
|------------|-----------|------------|
| 0.0 - 0.1 | Normal | No intervention |
| 0.1 - 0.3 | Moderate slip | Light torque reduction |
| > 0.3 | Excessive slip | Heavy torque reduction |

### Control Algorithm

```cpp
double TractionControl::regulateTorque(Wheel& wheel, double requestedTorque,
                                       double slipSetpoint,
                                       const Vector2d& wheelVelocityLocal,
                                       double dt) {
    // Only intervene during acceleration
    if (requestedTorque <= 0.0) {
        interferencePercent = 0.0;
        return requestedTorque;
    }

    double slipRatio = wheel.calculateSlipRatio(wheelVelocityLocal);
    double error = slipSetpoint - slipRatio;
    double changeInSlip = slipRatio - wheel.previousSlipError;

    // PD control
    double adjustedTorque = requestedTorque + kp * error - kd * changeInSlip;

    // Track intervention level
    double reduction = requestedTorque - adjustedTorque;
    if (reduction > 0) {
        interferencePercent = (reduction / requestedTorque) * 100.0;
    }

    wheel.previousSlipError = slipRatio;
    return adjustedTorque;
}
```

### Setpoint Selection

The slip setpoint is `0.1` (10% slip ratio) because:
- Peak tire grip occurs at 5-15% slip
- Below 5%: Not using full grip potential
- Above 15%: Tire is sliding, reduced grip

### Control Gains

| Parameter | Value | Effect |
|-----------|-------|--------|
| `TIRE_TCS_kP` | 2.0 | Torque reduction per unit slip error |
| `TIRE_TCS_kD` | 0.5 | Damping of slip oscillations |

**Tuning considerations:**
- Higher Kp → Faster response but potential oscillation
- Higher Kd → Smoother but slower response

### Top Speed Bypass

TCS is disabled near top speed to allow full power:

```cpp
if (wheel.angular_velocity * wheel.wheelRadius >= CAR_TOP_SPEED) {
    wheel.tcsInterference = 0.0;
}
```

## Anti-lock Braking System (ABS)

### Purpose

ABS prevents wheel lockup during braking by modulating brake pressure when wheels slip excessively.

### Why Prevent Lockup?

| Condition | Friction | Steering | Stopping Distance |
|-----------|----------|----------|-------------------|
| Rolling | Peak | Full | Optimal |
| Locked | Kinetic (lower) | None | Longer |

A locked wheel:
1. Has less friction than a rolling wheel
2. Cannot generate steering forces
3. Causes flat spots on the tire

### Control Algorithm

```cpp
double AntiLockBrakes::regulateBrakePressure(Wheel& wheel, double requestedBrakeTorque,
                                              double slipSetpoint,
                                              const Vector2d& wheelVelocityLocal,
                                              double vehicleSpeed, double dt) {
    // Don't fight a stopped wheel
    if (abs(wheel.angular_velocity) < 1e-3) {
        wheel.angular_velocity = 0.0;
        return 0.0;
    }

    // Low-speed bypass for final stopping
    if (vehicleSpeed < 0.1) {
        return -abs(requestedBrakeTorque) * copysign(1.0, wheel.angular_velocity);
    }

    double slipRatio = wheel.calculateSlipRatio(wheelVelocityLocal);
    double error = slipSetpoint - slipRatio;
    double changeInSlip = slipRatio - wheel.previousAbsSlipError;

    // Base brake torque (opposite to wheel rotation)
    double baseBrakeTorque = -abs(requestedBrakeTorque) * copysign(1.0, wheel.angular_velocity);

    // PD control adjustment
    double adjustedBrakeTorque = baseBrakeTorque + kp * error - kd * changeInSlip;

    // Prevent accelerating the wheel
    if (wheel.angular_velocity > 0 && adjustedBrakeTorque > 0) {
        adjustedBrakeTorque = 0.0;
    }

    wheel.previousAbsSlipError = slipRatio;
    return adjustedBrakeTorque;
}
```

### Negative Slip Setpoint

The ABS setpoint is `-0.2` because during braking:
- Slip ratio is negative (wheel slower than vehicle)
- -20% slip maintains steering while maximizing braking
- More aggressive than TCS because braking is time-critical

### Low-Speed Handling

At very low speeds (<0.1 m/s), ABS disengages to allow complete stop:

```cpp
if (vehicleSpeed < 0.1) {
    return -abs(requestedBrakeTorque) * copysign(1.0, wheel.angular_velocity);
}
```

Without this, ABS would prevent the final stop.

### Control Gains

| Parameter | Value | Effect |
|-----------|-------|--------|
| `ABS_kP` | 2.0 | Brake release per unit slip error |
| `ABS_kD` | 0.5 | Damping of pressure oscillations |

## Telemetry Integration

Both systems report intervention levels to the UI:

```cpp
wheel.tcsInterference = interferencePercent;  // 0-100%
wheel.absInterference = interferencePercent;  // 0-100%
```

This allows real-time visualization of when and how much the systems are working.

## System Comparison

| Aspect | TCS | ABS |
|--------|-----|-----|
| **Trigger** | Positive slip ratio | Negative slip ratio |
| **Setpoint** | +0.1 (10%) | -0.2 (20%) |
| **Actuator** | Engine torque | Brake torque |
| **Goal** | Prevent wheelspin | Prevent lockup |
| **Priority** | Performance | Safety |

## Design Decisions

### Why Torque Reduction (not Cut)?

Real TCS systems cut fuel/spark entirely. This simulation uses proportional reduction for:
- Smoother driving experience
- More predictable behavior
- Easier tuning

### Why Symmetric Gains?

Both systems use identical Kp/Kd values. This simplification works because:
- Same underlying slip dynamics
- Similar response time requirements
- Easier to tune and maintain

### Independent Wheel Control

Each wheel has its own slip history:

```cpp
wheel.previousSlipError = slipRatio;
wheel.previousAbsSlipError = slipRatio;
```

This allows proper handling of:
- Split-μ surfaces (ice on one side)
- Uneven weight distribution
- Individual wheel issues

---

**Previous: [Vehicle Dynamics](04-vehicle-dynamics.md)** | **Next: [Design Tradeoffs](06-tradeoffs.md)**
