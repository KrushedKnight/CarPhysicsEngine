
#ifndef SIMPLETRAFFICGAME_ENGINE_H
#define SIMPLETRAFFICGAME_ENGINE_H

#endif


class Engine
{
private:
    double rpm{1000};
    double loadTorque{0};
    double engineTorque{0};
    double currentPower{0};
    double currentVolumetricEfficiency{0.8};
    double currentAirFlowRate{0};
    double getVolumetricEfficiency();
    double getAirFlowRate(double throttle);
    double getAirFuelRatio();
    double getPowerGenerated(double throttle);


public:
    void updateRPM(double throttle, double effectiveInertia);
    double getRPM() const;
    double calculateTorque(double throttle);
    void addLoadTorque(double torque);

    double getEngineTorque() const;
    double getLoadTorque() const;
    double getCurrentPower() const;
    double getAirFuelRatioValue() const;
    double getVolumetricEfficiencyValue() const;
    double getAirFlowRateValue() const;
    double getPowerGeneratedValue(double throttle) const;
};