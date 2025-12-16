#include <gtest/gtest.h>
#include "../Car.h"
#include "../constants.h"
#include <Eigen/Dense>
#include <cmath>

class CarTest : public ::testing::Test {
protected:
    Car* car;

    void SetUp() override {
        car = new Car(100.0, 100.0, 25, 45);
    }

    void TearDown() override {
        delete car;
    }
};

TEST_F(CarTest, ConstructorInitializesCorrectly) {
    EXPECT_DOUBLE_EQ(car->pos_x, 100.0);
    EXPECT_DOUBLE_EQ(car->pos_y, 100.0);
    EXPECT_EQ(car->getWidth(), 25);
    EXPECT_EQ(car->getHeight(), 45);
    EXPECT_DOUBLE_EQ(car->steering_angle, 0.0);
    EXPECT_DOUBLE_EQ(car->engine_power, Constants::CAR_POWER);
    EXPECT_DOUBLE_EQ(car->braking_power, Constants::BRAKING_POWER);
}

TEST_F(CarTest, ConstructorCreatesAllWheels) {
    EXPECT_NE(car->frontLeft, nullptr);
    EXPECT_NE(car->frontRight, nullptr);
    EXPECT_NE(car->backLeft, nullptr);
    EXPECT_NE(car->backRight, nullptr);
    EXPECT_EQ(car->wheels.size(), 4);
}

TEST_F(CarTest, WheelsArrayContainsCorrectWheels) {
    EXPECT_EQ(car->wheels[0], car->frontLeft);
    EXPECT_EQ(car->wheels[1], car->frontRight);
    EXPECT_EQ(car->wheels[2], car->backLeft);
    EXPECT_EQ(car->wheels[3], car->backRight);
}

TEST_F(CarTest, ApplySteeringChangesSteeringAngle) {
    double initialAngle = car->steering_angle;
    car->applySteering(50.0);

    EXPECT_DOUBLE_EQ(car->steering_angle, initialAngle + 50.0);
}

TEST_F(CarTest, ApplySteeringNegativeAmount) {
    car->applySteering(-50.0);
    EXPECT_DOUBLE_EQ(car->steering_angle, -50.0);
}

TEST_F(CarTest, ApplySteeringClampsAtMaximum) {
    car->applySteering(1000.0);
    EXPECT_DOUBLE_EQ(car->steering_angle, Constants::MAX_STEERING_ANGLE);
}

TEST_F(CarTest, ApplySteeringClampsAtMinimum) {
    car->applySteering(-1000.0);
    EXPECT_DOUBLE_EQ(car->steering_angle, -Constants::MAX_STEERING_ANGLE);
}

TEST_F(CarTest, ApplySteeringUpdatesWheelAngles) {
    car->applySteering(50.0);

    double expectedWheelAngle = 50.0 * Constants::STEERING_RACK;
    EXPECT_DOUBLE_EQ(car->frontLeft->wheelAngle, expectedWheelAngle);
    EXPECT_DOUBLE_EQ(car->frontRight->wheelAngle, expectedWheelAngle);
}

TEST_F(CarTest, ApplySteeringDoesNotAffectRearWheels) {
    car->applySteering(50.0);

    EXPECT_DOUBLE_EQ(car->backLeft->wheelAngle, 0.0);
    EXPECT_DOUBLE_EQ(car->backRight->wheelAngle, 0.0);
}

TEST_F(CarTest, MultipleSteeringInputsAccumulate) {
    car->applySteering(30.0);
    car->applySteering(20.0);

    EXPECT_DOUBLE_EQ(car->steering_angle, 50.0);
}

TEST_F(CarTest, ApplyForceFeedbackReducesSteeringAngle) {
    car->applySteering(100.0);
    double initialAngle = car->steering_angle;

    car->applyForceFeedback();

    EXPECT_LT(car->steering_angle, initialAngle);
    EXPECT_GT(car->steering_angle, 0.0);
}

TEST_F(CarTest, ApplyForceFeedbackPreservesSign) {
    car->applySteering(-100.0);
    car->applyForceFeedback();

    EXPECT_LT(car->steering_angle, 0.0);
}

TEST_F(CarTest, ApplyForceFeedbackAppliesDecayFactor) {
    car->applySteering(100.0);
    double initialAngle = car->steering_angle;

    car->applyForceFeedback();

    double expectedAngle = initialAngle * Constants::FORCE_FEEDBACK_DECAY;
    EXPECT_DOUBLE_EQ(car->steering_angle, expectedAngle);
}

TEST_F(CarTest, ApplyForceFeedbackUpdatesWheelAngles) {
    car->applySteering(100.0);
    car->applyForceFeedback();

    double expectedWheelAngle = car->steering_angle * Constants::STEERING_RACK;
    EXPECT_DOUBLE_EQ(car->frontLeft->wheelAngle, expectedWheelAngle);
    EXPECT_DOUBLE_EQ(car->frontRight->wheelAngle, expectedWheelAngle);
}

TEST_F(CarTest, ApplyEngineTorqueIncreasesWheelTorque) {
    car->applyEngineTorque();

    EXPECT_NE(car->frontLeft->angular_torque, 0.0);
    EXPECT_NE(car->frontRight->angular_torque, 0.0);
    EXPECT_NE(car->backLeft->angular_torque, 0.0);
    EXPECT_NE(car->backRight->angular_torque, 0.0);
}

TEST_F(CarTest, ApplyEngineTorqueRespectsTopSpeed) {
    car->frontLeft->angular_velocity = Constants::CAR_TOP_SPEED / car->frontLeft->wheelRadius + 10.0;

    double initialTorque = car->frontLeft->angular_torque;
    car->applyEngineTorque();

    EXPECT_DOUBLE_EQ(car->frontLeft->angular_torque, initialTorque);
}

TEST_F(CarTest, ApplyEngineTorqueMultipleTimesAccumulatesTorque) {
    car->applyEngineTorque();
    double torque1 = car->frontLeft->angular_torque;

    car->applyEngineTorque();
    double torque2 = car->frontLeft->angular_torque;

    EXPECT_DOUBLE_EQ(torque2, 2.0 * torque1);
}

TEST_F(CarTest, ApplyBrakesWithPositiveVelocity) {
    car->frontLeft->angular_velocity = 10.0;
    car->applyBrakes();

    EXPECT_LT(car->frontLeft->angular_torque, 0.0);
}

TEST_F(CarTest, ApplyBrakesWithNegativeVelocity) {
    car->frontLeft->angular_velocity = -10.0;
    car->applyBrakes();

    EXPECT_GT(car->frontLeft->angular_torque, 0.0);
}

TEST_F(CarTest, ApplyBrakesWithZeroVelocityDoesNothing) {
    car->frontLeft->angular_velocity = 0.0;
    car->applyBrakes();

    EXPECT_DOUBLE_EQ(car->frontLeft->angular_torque, 0.0);
}

TEST_F(CarTest, ApplyBrakesAppliesToAllWheels) {
    car->frontLeft->angular_velocity = 10.0;
    car->frontRight->angular_velocity = 10.0;
    car->backLeft->angular_velocity = 10.0;
    car->backRight->angular_velocity = 10.0;

    car->applyBrakes();

    EXPECT_LT(car->frontLeft->angular_torque, 0.0);
    EXPECT_LT(car->frontRight->angular_torque, 0.0);
    EXPECT_LT(car->backLeft->angular_torque, 0.0);
    EXPECT_LT(car->backRight->angular_torque, 0.0);
}

TEST_F(CarTest, SumWheelForcesAppliesForces) {
    car->velocity = Eigen::Vector2d(10.0, 0.0);

    car->sumWheelForces();

    EXPECT_NE(car->forces, Eigen::Vector2d::Zero());
}

TEST_F(CarTest, SumWheelForcesAppliesTorque) {
    car->velocity = Eigen::Vector2d(0.0, 10.0);
    car->applySteering(50.0);

    car->sumWheelForces();

    EXPECT_NO_THROW(car->sumWheelForces());
}

TEST_F(CarTest, SumWheelForcesWithZeroVelocity) {
    car->velocity = Eigen::Vector2d::Zero();
    car->sumWheelForces();

    EXPECT_NO_THROW(car->sumWheelForces());
}

TEST_F(CarTest, MoveWheelsIncrementsWheelTime) {
    double initialPosition = car->frontLeft->angular_position;

    car->frontLeft->angular_velocity = 10.0;

    car->moveWheels();

    EXPECT_NE(car->frontLeft->angular_position, initialPosition);
}

TEST_F(CarTest, MoveWheelsAppliesForceFeedback) {
    car->applySteering(100.0);
    double initialAngle = car->steering_angle;

    car->moveWheels();

    EXPECT_LT(car->steering_angle, initialAngle);
}

TEST_F(CarTest, MoveWheelsUpdatesAllWheels) {
    car->frontLeft->angular_velocity = 10.0;
    car->frontRight->angular_velocity = 10.0;
    car->backLeft->angular_velocity = 10.0;
    car->backRight->angular_velocity = 10.0;

    double fl_pos = car->frontLeft->angular_position;
    double fr_pos = car->frontRight->angular_position;
    double bl_pos = car->backLeft->angular_position;
    double br_pos = car->backRight->angular_position;

    car->moveWheels();

    EXPECT_NE(car->frontLeft->angular_position, fl_pos);
    EXPECT_NE(car->frontRight->angular_position, fr_pos);
    EXPECT_NE(car->backLeft->angular_position, bl_pos);
    EXPECT_NE(car->backRight->angular_position, br_pos);
}

TEST_F(CarTest, GetAngleToWheelReturnsCorrectValue) {
    car->applySteering(50.0);
    double angle = car->getAngleToWheel(car->frontLeft);

    double expected = car->steering_angle * Constants::STEERING_RACK;
    EXPECT_DOUBLE_EQ(angle, expected);
}

TEST_F(CarTest, IntegrationSteeringAndMovement) {
    car->applySteering(50.0);

    car->applyEngineTorque();

    car->moveWheels();

    car->sumWheelForces();

    car->incrementTime(Constants::TIME_INTERVAL);

    EXPECT_NE(car->pos_x, 100.0);
}

TEST_F(CarTest, IntegrationBrakingReducesSpeed) {
    car->velocity = Eigen::Vector2d(10.0, 0.0);

    for (auto* wheel : car->wheels) {
        wheel->angular_velocity = 20.0;
    }

    double initialSpeed = car->velocity.norm();

    for (int i = 0; i < 10; i++) {
        car->applyBrakes();
        car->moveWheels();
        car->sumWheelForces();
        car->incrementTime(Constants::TIME_INTERVAL);
    }

    double finalSpeed = car->velocity.norm();
    EXPECT_LE(finalSpeed, initialSpeed * 1.1);
}

TEST_F(CarTest, IntegrationSteeringReturnsToCenter) {
    car->applySteering(100.0);
    double initialAngle = car->steering_angle;

    for (int i = 0; i < 50; i++) {
        car->moveWheels();
    }

    EXPECT_LT(car->steering_angle, initialAngle * 0.1);
}

TEST_F(CarTest, ZeroSizeCarDoesNotCrash) {
    Car* smallCar = new Car(0.0, 0.0, 0, 0);
    EXPECT_NO_THROW(smallCar->sumWheelForces());
    delete smallCar;
}

TEST_F(CarTest, NegativePositionIsValid) {
    Car* negCar = new Car(-100.0, -100.0, 25, 45);
    EXPECT_DOUBLE_EQ(negCar->pos_x, -100.0);
    EXPECT_DOUBLE_EQ(negCar->pos_y, -100.0);
    delete negCar;
}
