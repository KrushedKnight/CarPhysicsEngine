#include "vehicle/Engine.h"

#include "config/Constants.h"
#include "config/EngineConstants.h"
#include "config/PhysicsConstants.h"
#include "config/RenderingConstants.h"


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

void Engine::updateRPM(double throttle, double netTorque)
{
    rpm = rpm + (netTorque / EngineConstants::ENGINE_MOMENT_OF_INERTIA) * (30 / M_PI) * PhysicsConstants::TIME_INTERVAL;
}

double Engine::getRPM()
{
    return rpm;
}

double Engine::calculateTorque(double throttle)
{
    double angularSpeed = (2.0 * M_PI * rpm) / 60.0;
    double driveshaftTorque = getPowerGenerated(throttle) / angularSpeed;
    return driveshaftTorque;
}


