#include "vehicle/Gearbox.h"

#include "config/PhysicsConstants.h"
#include "vehicle/Engine.h"
#include <iostream>

Gearbox::Gearbox(const std::vector<double>& ratios, double finalDriveRatio)
{
    this->gearRatios = ratios;
    this->finalDrive = finalDriveRatio;
    this->clutchPressed = false;
    this->selectedGear = -1;
    this->clutchEngagement = 1.0;
    this->loadTorque = 0.0;
    this->engineTorque = 0.0;
    this->clutchTorque = 0.0;
    this->clutchSlip = 0.0;
    this->heldTorque = 0.0;
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
    return this->getGearRatio();
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
        gearRatio = -1.0 / (this->gearRatios[0] * finalDrive);
    }
    else
    {
        gearRatio = 1.0 / (this->gearRatios[this->selectedGear] * finalDrive);
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
    static int callCounter = 0;
    if (callCounter++ % 60 == 0) {
        std::cout << ">>> convertEngineTorqueToWheel CALLED | Gear: " << selectedGear
                  << " | ClutchPressed: " << clutchPressed << std::endl;
    }

    if (selectedGear == -1 || clutchPressed)
    {
        this->clutchTorque = 0.0;
        this->heldTorque = 0.0;

        if (selectedGear == -1) {
            this->clutchSlip = 0.0;
        } else {
            double engineOmega = (2.0 * M_PI * engine->getRPM()) / 60.0;
            double transOmega = wheelOmega * wheelToEngineRatio();
            this->clutchSlip = engineOmega - transOmega;
        }
        return 0.0;
    }

    double bite = calculateBite();

    double engineOmega = (2.0 * M_PI * engine->getRPM()) / 60.0;
    double transOmega = wheelOmega * wheelToEngineRatio();
    double slip = engineOmega - transOmega;

    static int debugCounter = 0;
    if (debugCounter++ % 10 == 0) {
        std::cout << "EngineRPM: " << engine->getRPM()
                  << " | WheelOmega: " << wheelOmega
                  << " | EngineOmega: " << engineOmega
                  << " | TransOmega: " << transOmega
                  << " | Slip: " << slip
                  << " | Bite: " << bite << std::endl;
    }

    this->heldTorque = engineTorque;
    double torqueClutch;

    if (bite >= 0.95 ) {
        double lockingK = PhysicsConstants::CLUTCH_SLIP_K * 50.0;
        double dampingK = lockingK * 0.1;

        double slipRate = (slip - this->clutchSlip) / PhysicsConstants::TIME_INTERVAL;
        torqueClutch = slip * lockingK + slipRate * dampingK;
        torqueClutch = std::clamp(torqueClutch, -PhysicsConstants::CLUTCH_MAX_TORQUE, PhysicsConstants::CLUTCH_MAX_TORQUE);
    } else {
        double torqueMax = bite * PhysicsConstants::CLUTCH_MAX_TORQUE;
        torqueClutch = std::clamp(slip * PhysicsConstants::CLUTCH_SLIP_K, -torqueMax, torqueMax);
    }

    this->engineTorque = engineTorque - torqueClutch;
    this->clutchTorque = torqueClutch;
    this->clutchSlip = slip;

    return torqueClutch / engineToWheelRatio();
}

double Gearbox::convertWheelTorqueToEngine(double wheelTorque)
{
    if (selectedGear == -1 || clutchPressed)
    {
        return 0.0;
    }
    loadTorque = wheelTorque / wheelToEngineRatio();
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

double Gearbox::getClutchEngagement() const
{
    return clutchEngagement;
}

double Gearbox::getClutchTorque() const
{
    return clutchTorque;
}

double Gearbox::getClutchSlip() const
{
    return clutchSlip;
}

