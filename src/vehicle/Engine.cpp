#include "vehicle/Engine.h"

#include "config/Constants.h"
#include "config/EngineConstants.h"
#include "config/PhysicsConstants.h"
#include "config/RenderingConstants.h"

void Engine::addLoadTorque(double torque)
{
    loadTorque += torque;
}
double Engine::getVolumetricEfficiency()
{
    double peakRPM = 5000.0;
    double rpmRatio = rpm / peakRPM;

    if (rpm < peakRPM) {
        return 0.8 * (0.5 + 0.5 * rpmRatio);
    } else {
        return 0.8 / (1.0 + 0.3 * (rpmRatio - 1.0));
    }
}


double Engine::getAirFlowRate(double throttle)
{
    double airDensity = EngineConstants::INTAKE_MANIFOLD_PRESSURE / (EngineConstants::R_AIR * EngineConstants::AIR_TEMP);
    double airMassPerCycle = getVolumetricEfficiency() * airDensity * EngineConstants::CYLINDER_VOLUME * throttle;
    return airMassPerCycle * (rpm / 120.0);
}

double Engine::getAirFuelRatio()
{
    return 14.7;
}

double Engine::getPowerGenerated(double throttle)
{
    double fuelMass = getAirFlowRate(throttle) / getAirFuelRatio();
    double powerGenerated = fuelMass * EngineConstants::LATENT_HEAT * EngineConstants::ENGINE_EFFICIENCY;
    return powerGenerated;
}

void Engine::updateRPM(double throttle, double effectiveInertia)
{
    double frictionTorque = EngineConstants::ENGINE_FRICTION_COEFFICIENT * rpm;
    double netTorque = engineTorque - loadTorque - frictionTorque;
    rpm += (netTorque / effectiveInertia) * (30 / M_PI) * PhysicsConstants::TIME_INTERVAL;

    rpm = std::clamp(rpm, 0.0, 8000.0);

    loadTorque = 0;
}

double Engine::getRPM() const
{
    return rpm;
}

double Engine::calculateTorque(double throttle)
{
    double effectiveThrottle = throttle;

    if (rpm >= 7800.0) {
        double excessRPM = rpm - 7800.0;
        double reductionFactor = std::max(0.0, 1.0 - (excessRPM / 200.0));
        effectiveThrottle *= reductionFactor;
    } else if (throttle < 0.01) {
        effectiveThrottle = 0.05;
    }

    currentVolumetricEfficiency = getVolumetricEfficiency();
    currentAirFlowRate = getAirFlowRate(effectiveThrottle);
    currentPower = getPowerGenerated(effectiveThrottle);

    double angularSpeed = (2.0 * M_PI * rpm) / 60.0;
    engineTorque = currentPower / angularSpeed;
    return engineTorque;
}

double Engine::getEngineTorque() const
{
    return engineTorque;
}

double Engine::getLoadTorque() const
{
    return loadTorque;
}

double Engine::getCurrentPower() const
{
    return currentPower;
}

double Engine::getAirFuelRatioValue() const
{
    return 14.7;
}

double Engine::getVolumetricEfficiencyValue() const
{
    return currentVolumetricEfficiency;
}

double Engine::getAirFlowRateValue() const
{
    return currentAirFlowRate;
}

double Engine::getPowerGeneratedValue(double throttle) const
{
    double airFlowRate = getAirFlowRateValue();
    double fuelMass = airFlowRate / 14.7;
    double powerGenerated = fuelMass * EngineConstants::LATENT_HEAT * EngineConstants::ENGINE_EFFICIENCY;
    return powerGenerated;
}


