#ifndef SIMPLETRAFFICGAME_GEARBOX_H
#define SIMPLETRAFFICGAME_GEARBOX_H

#include <vector>

class Engine;

class Gearbox {
private:
    int selectedGear;
    std::vector<double> gearRatios;
    double finalDrive;
    bool clutchEngaged;

public:
    Gearbox(const std::vector<double>& ratios, double finalDriveRatio);

    double engineToWheelRatio();
    double wheelToEngineRatio();

    bool isClutchHeld() const;
    void holdClutch();
    void releaseClutch();

    bool shiftUp();
    bool shiftDown();

    int getCurrentGear() const;
    double getGearRatio() const;
};

#endif