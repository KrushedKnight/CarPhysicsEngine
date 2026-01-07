# Drivetrain System

## Overview

The drivetrain converts engine power into wheel torque through a realistic simulation of internal combustion engine physics, clutch mechanics, and gearbox ratios. This system models the complex interaction between spinning components with different inertias.

## Engine Simulation

### Thermodynamic Model

The engine uses first-principles thermodynamics rather than lookup tables:

```
Power = (Air Mass × Fuel Energy × Efficiency) / Time
```

#### Air Flow Calculation

```cpp
double airDensity = INTAKE_PRESSURE / (R_AIR * AIR_TEMP);
double airMassPerCycle = volumetricEfficiency * airDensity * CYLINDER_VOLUME * throttle;
double airFlowRate = airMassPerCycle * (rpm / 120.0);
```

Where:
- `INTAKE_PRESSURE = 101325 Pa` (atmospheric)
- `R_AIR = 287 J/(kg·K)` (gas constant for air)
- `AIR_TEMP = 298 K` (25°C)
- `CYLINDER_VOLUME = 0.005 m³` (5L displacement)

#### Volumetric Efficiency

Real engines breathe better at certain RPMs due to intake/exhaust tuning:

```cpp
double getVolumetricEfficiency() {
    double peakRPM = 5000.0;
    double rpmRatio = rpm / peakRPM;

    if (rpm < peakRPM) {
        return 0.8 * (0.5 + 0.5 * rpmRatio);  // Rising efficiency
    } else {
        return 0.8 / (1.0 + 0.3 * (rpmRatio - 1.0));  // Falling efficiency
    }
}
```

This creates a characteristic torque curve that peaks mid-range.

#### Power to Torque

```cpp
double angularSpeed = (2.0 * PI * rpm) / 60.0;
engineTorque = currentPower / angularSpeed;
```

### RPM Dynamics

Engine RPM changes based on net torque:

```cpp
double frictionTorque = ENGINE_FRICTION_COEFFICIENT * rpm;
double netTorque = engineTorque - loadTorque - frictionTorque;
rpm += (netTorque / effectiveInertia) * (30 / PI) * TIME_INTERVAL;
```

The friction term (`0.02 × RPM`) provides natural engine braking and prevents infinite acceleration.

### Rev Limiter

Soft cutoff at high RPM:

```cpp
if (rpm >= 7800.0) {
    double excessRPM = rpm - 7800.0;
    double reductionFactor = max(0.0, 1.0 - (excessRPM / 200.0));
    effectiveThrottle *= reductionFactor;
}
```

This progressively reduces power from 7800-8000 RPM, simulating a fuel cut rev limiter.

### Idle Control

Maintains minimum RPM when throttle is released:

```cpp
if (throttle < 0.01) {
    effectiveThrottle = 0.05;  // Minimum throttle for idle
}
```

## Gearbox

### Gear Ratios

The transmission uses a 6-speed gearbox plus reverse:

| Gear | Ratio | Total Ratio (with 4.2 final drive) |
|------|-------|-----------------------------------|
| R | 3.5:1 | 14.7:1 |
| 1 | 3.5:1 | 14.7:1 |
| 2 | 2.2:1 | 9.24:1 |
| 3 | 1.5:1 | 6.30:1 |
| 4 | 1.0:1 | 4.20:1 |
| 5 | 0.75:1 | 3.15:1 |
| 6 | 0.6:1 | 2.52:1 |

Lower gears multiply torque but limit top speed; higher gears do the opposite.

### Clutch Physics

The clutch is modeled as a slip-based torque coupler:

#### Engagement States

```cpp
double calculateBite() {
    if (clutchEngagement < 0.6)
        return 0.0;           // Fully disengaged
    else if (clutchEngagement > 0.9)
        return 1.0;           // Fully engaged
    else
        return (clutchEngagement - 0.6) / 0.3;  // Friction zone
}
```

The "bite point" occurs at 60% pedal release, mimicking real clutch behavior.

#### Slip-Based Torque

```cpp
double engineOmega = (2.0 * PI * engine->getRPM()) / 60.0;
double transOmega = wheelOmega * wheelToEngineRatio();
double slip = engineOmega - transOmega;
```

**When engaging (bite < lock threshold):**
```cpp
double torqueMax = bite * CLUTCH_MAX_TORQUE;
targetTorque = clamp(slip * CLUTCH_SLIP_K, -torqueMax, torqueMax);
```

**When locked (bite ≥ 0.70):**
```cpp
double lockingK = CLUTCH_SLIP_K * 3.0;
double dampingK = lockingK * 0.5;
double slipRate = (slip - previousSlip) / TIME_INTERVAL;
targetTorque = slip * lockingK + slipRate * dampingK;
```

The locked state uses PD control to minimize slip while damping oscillations.

#### Torque Smoothing

To prevent jerky engagement:

```cpp
double smoothing = 0.12;
torqueClutch = previousTorque + smoothing * (targetTorque - previousTorque);
```

This creates smooth clutch engagement over approximately 8 frames.

### Reflected Inertia

When the clutch is engaged, the engine "feels" the wheel inertia and vice versa:

```cpp
double getReflectedEngineInertia(double engineInertia) {
    double ratio = wheelToEngineRatio();
    return engineInertia * ratio * ratio;
}
```

In first gear (14.7:1 total ratio), engine inertia appears 216× larger at the wheels!

This is why:
- Launching in first gear feels sluggish
- Higher gears feel more responsive
- Downshifting causes engine RPM to spike

### Torque Path

```
Engine Torque (Nm)
       │
       ▼
   ┌───────┐
   │ Clutch │ ─── Slip losses
   └───┬───┘
       │
       ▼
   ┌───────┐
   │Gearbox│ ×gear ratio
   └───┬───┘
       │
       ▼
   ┌────────┐
   │Final Dr│ ×4.2
   └───┬────┘
       │
       ▼
  Wheel Torque (Nm)
```

## Engine Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `CYLINDER_VOLUME` | 0.005 m³ | 5L displacement |
| `MAX_RPM` | 8000 | Rev limit |
| `LATENT_HEAT` | 44 MJ/kg | Gasoline energy density |
| `ENGINE_EFFICIENCY` | 0.64 | Thermal efficiency |
| `ENGINE_MOMENT_OF_INERTIA` | 1.6 kg·m² | Flywheel + rotating assembly |
| `ENGINE_FRICTION_COEFFICIENT` | 0.02 | Internal friction |

## Clutch Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `CLUTCH_MAX_TORQUE` | 750 Nm | Maximum transmittable torque |
| `CLUTCH_SLIP_K` | 4.5 | Slip-to-torque coefficient |
| `CLUTCH_LOCK_THRESHOLD` | 0.70 | Bite level for locking mode |

## Automatic Stall Prevention

If RPM drops below 800 in gear, the system auto-downshifts to neutral:

```cpp
if (engine.getRPM() < 800.0 && getCurrentGear() != -1) {
    gearbox.holdClutch();
    while (getCurrentGear() > -1) {
        gearbox.shiftDown();
    }
    gearbox.releaseClutch();
}
```

This prevents the engine from stalling during aggressive braking.

---

**Previous: [Tire Physics](02-tire-model.md)** | **Next: [Vehicle Dynamics](04-vehicle-dynamics.md)**
