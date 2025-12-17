#include "vehicle/Gearbox.h"

#include "vehicle/Engine.h"

Gearbox::Gearbox(const std::vector<double>& ratios, double finalDriveRatio)
{
    this->gearRatios = ratios;
    this->finalDrive = finalDriveRatio;
    this->clutchEngaged = false;
    this->selectedGear = -1;
}

bool Gearbox::isClutchHeld() const
{
    return clutchEngaged;
}

void Gearbox::holdClutch()
{
    clutchEngaged = true;
}

void Gearbox::releaseClutch()
{
    clutchEngaged = false;
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


