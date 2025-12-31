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

double Gearbox::wheelToEngineRatio() const
{
    return 1.0 / getGearRatio();
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
    if (!clutchPressed)
    {
        return false;
    }

    if (selectedGear > -2)
    {
        selectedGear--;
        return true;
    }
    return false;
}

bool Gearbox::shiftUp()
{
    if (!clutchPressed)
    {
        return false;
    }

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
        std::cout << "\n=== CLUTCH DEBUG ===" << std::endl;
        std::cout << "EngineRPM: " << engine->getRPM() << " | EngineOmega: " << engineOmega << " rad/s" << std::endl;
        std::cout << "WheelOmega: " << wheelOmega << " rad/s | TransOmega: " << transOmega << " rad/s" << std::endl;
        std::cout << "Slip: " << slip << " rad/s | SlipSign: " << (slip > 0 ? "POS (engine driving)" : "NEG (wheels driving)") << std::endl;
        std::cout << "Bite: " << bite << " | GearRatio: " << (1.0 / engineToWheelRatio()) << ":1" << std::endl;
        std::cout << "wheelToEngineRatio: " << wheelToEngineRatio() << std::endl;
    }

    this->heldTorque = engineTorque;
    double targetTorque;

    if (bite >= PhysicsConstants::CLUTCH_LOCK_THRESHOLD) {
        double lockingK = PhysicsConstants::CLUTCH_SLIP_K * 3.0;
        double dampingK = lockingK * 0.5;

        double slipRate = (slip - this->clutchSlip) / PhysicsConstants::TIME_INTERVAL;
        slipRate = std::clamp(slipRate, -500.0, 500.0);
        targetTorque = slip * lockingK + slipRate * dampingK;
        targetTorque = std::clamp(targetTorque, -PhysicsConstants::CLUTCH_MAX_TORQUE, PhysicsConstants::CLUTCH_MAX_TORQUE);
    } else {
        double torqueMax = bite * PhysicsConstants::CLUTCH_MAX_TORQUE;
        targetTorque = std::clamp(slip * PhysicsConstants::CLUTCH_SLIP_K, -torqueMax, torqueMax);
    }

    double smoothing = 0.06;
    double torqueClutch;

    bool sameSign = (this->clutchTorque >= 0) == (targetTorque >= 0);
    if (sameSign || std::abs(this->clutchTorque) < 1.0) {
        torqueClutch = this->clutchTorque + smoothing * (targetTorque - this->clutchTorque);
    } else {
        torqueClutch = this->clutchTorque * (1.0 - smoothing);
    }

    if (std::isnan(torqueClutch) || std::isinf(torqueClutch)) {
        torqueClutch = 0.0;
    }

    this->engineTorque = engineTorque - torqueClutch;
    this->clutchTorque = torqueClutch;
    this->clutchSlip = slip;

    double ratio = engineToWheelRatio();
    double wheelTorque = (std::abs(ratio) > 1e-6) ? (torqueClutch / ratio) : 0.0;

    if (debugCounter % 10 == 1) {
        std::cout << "EngineInputTorque: " << engineTorque << " Nm" << std::endl;
        std::cout << "ClutchTorque: " << torqueClutch << " Nm | WheelTorque: " << wheelTorque << " Nm" << std::endl;
        std::cout << "LockingMode: " << (bite >= PhysicsConstants::CLUTCH_LOCK_THRESHOLD ? "YES" : "NO");
        if (bite >= PhysicsConstants::CLUTCH_LOCK_THRESHOLD) {
            std::cout << " | lockingK=" << (PhysicsConstants::CLUTCH_SLIP_K * 120.0);
        }
        std::cout << std::endl;
        std::cout << "===================" << std::endl;
    }

    return wheelTorque;
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

double Gearbox::getReflectedEngineInertia(double engineInertia) const
{
    if (selectedGear == -1 || clutchPressed) {
        return 0.0;
    }
    double ratio = wheelToEngineRatio();
    return engineInertia * ratio * ratio;
}

double Gearbox::getReflectedWheelInertia(double wheelInertia) const
{
    if (selectedGear == -1 || clutchPressed) {
        return 0.0;
    }
    double ratio = wheelToEngineRatio();
    return wheelInertia / (ratio * ratio);
}

