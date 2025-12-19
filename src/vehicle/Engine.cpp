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
    return 0.8; //placeholder- should be a function of rpm
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

void Engine::updateRPM(double throttle)
{
    double frictionTorque = EngineConstants::ENGINE_FRICTION_COEFFICIENT * rpm;
    double netTorque = engineTorque - loadTorque - frictionTorque;
    rpm = rpm + (netTorque / EngineConstants::ENGINE_MOMENT_OF_INERTIA) * (30 / M_PI) * PhysicsConstants::TIME_INTERVAL;

    loadTorque = 0;
}

double Engine::getRPM() const
{
    return rpm;
}

double Engine::calculateTorque(double throttle)
{
    double angularSpeed = (2.0 * M_PI * rpm) / 60.0;
    engineTorque = getPowerGenerated(throttle) / angularSpeed;
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

double Engine::getAirFuelRatioValue() const
{
    return 14.7;
}

double Engine::getVolumetricEfficiencyValue() const
{
    return 0.8;
}

double Engine::getAirFlowRateValue(double throttle) const
{
    double airDensity = EngineConstants::INTAKE_MANIFOLD_PRESSURE / (EngineConstants::R_AIR * EngineConstants::AIR_TEMP);
    double volumetricEfficiency = 0.8;
    double airMassPerCycle = volumetricEfficiency * airDensity * EngineConstants::CYLINDER_VOLUME * throttle;
    return airMassPerCycle * (rpm / 120.0);
}

double Engine::getPowerGeneratedValue(double throttle) const
{
    double airFlowRate = getAirFlowRateValue(throttle);
    double fuelMass = airFlowRate / 14.7;
    double powerGenerated = fuelMass * EngineConstants::LATENT_HEAT * EngineConstants::ENGINE_EFFICIENCY;
    return powerGenerated;
}


