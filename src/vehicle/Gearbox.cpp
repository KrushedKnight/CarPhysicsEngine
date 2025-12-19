#include "vehicle/Gearbox.h"

#include "config/PhysicsConstants.h"
#include "vehicle/Engine.h"

Gearbox::Gearbox(const std::vector<double>& ratios, double finalDriveRatio)
{
    this->gearRatios = ratios;
    this->finalDrive = finalDriveRatio;
    this->clutchPressed = false;
    this->selectedGear = -1;
    this->clutchEngagement = 1.0;
    this->loadTorque = 0.0;
    this->engineTorque = 0.0;
}

bool Gearbox::isClutchHeld() const
{
    return clutchPressed;
}

void Gearbox::holdClutch()
{
    clutchPressed = true;
}

void Gearbox::releaseClutch()
{
    clutchPressed = false;
}

double Gearbox::engineToWheelRatio()
{
    return (this->getGearRatio() * this->finalDrive);
}

double Gearbox::wheelToEngineRatio()
{
    return 1.0 / engineToWheelRatio();
}

int Gearbox::getCurrentGear() const
{
    return selectedGear;
}
double Gearbox::getGearRatio() const
{
    double gearRatio;
    if (this->selectedGear == -1)
    {
        gearRatio = 0;
    }
    else if(this->selectedGear == -2)
    {
        gearRatio = -1.0 / this->gearRatios[0];
    }
    else
    {
        gearRatio = 1.0 / this->gearRatios[this->selectedGear];
    }

    return gearRatio;
}

bool Gearbox::shiftDown()
{
    if (selectedGear > -2)
    {
        selectedGear--;
        return true;
    }
    return false;
}

bool Gearbox::shiftUp()
{
    if (selectedGear < static_cast<int>(this->gearRatios.size()) - 1)
    {
        selectedGear++;
        return true;
    }
    return false;
}

double Gearbox::calculateBite()
{
    double bite;
    if (clutchEngagement < 0.6)
        bite = 0.0;
    else if (clutchEngagement > 0.9)
        bite = 1.0;
    else
        bite = (clutchEngagement - 0.6) / (0.9 - 0.6);

    return bite;
}
double Gearbox::convertEngineTorqueToWheel(double engineTorque, Engine* engine, double wheelOmega)
{
    if (selectedGear == -1 || clutchPressed)
    {
        return 0.0;
    }

    double bite = calculateBite();
    double torqueMax = bite * PhysicsConstants::CLUTCH_MAX_TORQUE;


    double engineOmega = (2.0 * M_PI * engine->getRPM()) / 60.0;
    double transOmega = (wheelOmega * wheelToEngineRatio()); //double check this is the right one

    double slip = engineOmega - transOmega;
    double torqueClutch = std::clamp(slip * PhysicsConstants::CLUTCH_SLIP_K, -torqueMax, torqueMax);

    this->engineTorque = engineTorque - torqueClutch;

    return torqueClutch / engineToWheelRatio();
}

double Gearbox::convertWheelTorqueToEngine(double wheelTorque)
{
    if (selectedGear == -1 || clutchPressed)
    {
        return 0.0;
    }
    loadTorque = wheelTorque / engineToWheelRatio();
    return loadTorque;
}


double Gearbox::getEngineTorque()
{
    return engineTorque;
}

void Gearbox::update()
{
    double target = clutchPressed ? 0.0 : 1.0;
    double rate = target > clutchEngagement ? 12.0 : 6.0;
    clutchEngagement += (target - clutchEngagement) * rate * PhysicsConstants::TIME_INTERVAL;
}

