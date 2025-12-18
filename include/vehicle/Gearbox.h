#ifndef SIMPLETRAFFICGAME_GEARBOX_H
#define SIMPLETRAFFICGAME_GEARBOX_H

#include <vector>

class Engine;

class Gearbox {
private:
    int selectedGear;
    std::vector<double> gearRatios;
    double finalDrive;
    bool clutchPressed;
    double clutchEngagement;
    double loadTorque;
    double engineTorque;


//for tomorrow, this is all getting super super confusing needs updates to the system. basically loadTorque and engineTorque should be stored in gearbox class and gearbox should act on engine and load.

public:
    Gearbox(const std::vector<double>& ratios, double finalDriveRatio);

    double engineToWheelRatio();
    double wheelToEngineRatio();

    double convertEngineTorqueToWheel(double engineTorque, Engine* engine, double wheelOmega);
    double convertWheelTorqueToEngine(double wheelTorque);

    bool isClutchHeld() const;
    void holdClutch();
    void releaseClutch();

    bool shiftUp();
    bool shiftDown();

    int getCurrentGear() const;
    double getGearRatio() const;

    double calculateBite();
    void update();

    double getEngineTorque();
};

#endif