#include "vehicle/Engine.h"
#include "config/PhysicsConstants.h"

double Engine::calculateTorque()
{
    return (PhysicsConstants::CAR_POWER * PhysicsConstants::WHEEL_RADIUS);
}


