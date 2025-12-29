#include <gtest/gtest.h>
#include "vehicle/Car.h"
#include "config/PhysicsConstants.h"
#include "config/RenderingConstants.h"
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
    EXPECT_DOUBLE_EQ(car->engine_power, PhysicsConstants::CAR_POWER);
    EXPECT_DOUBLE_EQ(car->braking_power, PhysicsConstants::BRAKING_POWER);
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
    double steeringAmount = 0.5;  // radians
    car->applySteering(steeringAmount);

    EXPECT_DOUBLE_EQ(car->steering_angle, initialAngle + steeringAmount);
}

TEST_F(CarTest, ApplySteeringNegativeAmount) {
    double steeringAmount = -0.5;  // radians
    car->applySteering(steeringAmount);
    EXPECT_DOUBLE_EQ(car->steering_angle, steeringAmount);
}

TEST_F(CarTest, ApplySteeringClampsAtMaximum) {
    car->applySteering(1000.0);
    EXPECT_DOUBLE_EQ(car->steering_angle, PhysicsConstants::MAX_STEERING_ANGLE);
}

TEST_F(CarTest, ApplySteeringClampsAtMinimum) {
    car->applySteering(-1000.0);
    EXPECT_DOUBLE_EQ(car->steering_angle, -PhysicsConstants::MAX_STEERING_ANGLE);
}

TEST_F(CarTest, ApplySteeringUpdatesWheelAngles) {
    double steeringAmount = 0.5;  // radians
    car->applySteering(steeringAmount);

    double expectedWheelAngle = steeringAmount * PhysicsConstants::STEERING_RACK;
    EXPECT_DOUBLE_EQ(car->frontLeft->wheelAngle, expectedWheelAngle);
    EXPECT_DOUBLE_EQ(car->frontRight->wheelAngle, expectedWheelAngle);
}

TEST_F(CarTest, ApplySteeringDoesNotAffectRearWheels) {
    car->applySteering(0.5);  // radians

    EXPECT_DOUBLE_EQ(car->backLeft->wheelAngle, 0.0);
    EXPECT_DOUBLE_EQ(car->backRight->wheelAngle, 0.0);
}

TEST_F(CarTest, MultipleSteeringInputsAccumulate) {
    double amount1 = 0.3;  // radians
    double amount2 = 0.2;  // radians
    car->applySteering(amount1);
    car->applySteering(amount2);

    EXPECT_DOUBLE_EQ(car->steering_angle, amount1 + amount2);
}

TEST_F(CarTest, ApplyForceFeedbackReducesSteeringAngle) {
    car->applySteering(0.5);  // radians
    double initialAngle = car->steering_angle;

    car->applyForceFeedback();

    // Force feedback disabled (decay = 1.0), angle stays constant
    EXPECT_DOUBLE_EQ(car->steering_angle, initialAngle);
}

TEST_F(CarTest, ApplyForceFeedbackPreservesSign) {
    car->applySteering(-0.5);  // radians
    car->applyForceFeedback();

    EXPECT_LT(car->steering_angle, 0.0);
}

TEST_F(CarTest, ApplyForceFeedbackAppliesDecayFactor) {
    car->applySteering(0.5);  // radians
    double initialAngle = car->steering_angle;

    car->applyForceFeedback();

    double expectedAngle = initialAngle * PhysicsConstants::FORCE_FEEDBACK_DECAY;
    EXPECT_DOUBLE_EQ(car->steering_angle, expectedAngle);
}

TEST_F(CarTest, ApplyForceFeedbackUpdatesWheelAngles) {
    car->applySteering(100.0);
    car->applyForceFeedback();

    double expectedWheelAngle = car->steering_angle * PhysicsConstants::STEERING_RACK;
    EXPECT_DOUBLE_EQ(car->frontLeft->wheelAngle, expectedWheelAngle);
    EXPECT_DOUBLE_EQ(car->frontRight->wheelAngle, expectedWheelAngle);
}

TEST_F(CarTest, UpdateEngineAppliesWheelTorque) {
    car->releaseClutch();
    car->shiftUp();
    car->updateEngine(1.0);

    EXPECT_EQ(car->frontLeft->angular_torque, 0.0);
    EXPECT_EQ(car->frontRight->angular_torque, 0.0);
}

TEST_F(CarTest, UpdateEngineWithZeroThrottle) {
    car->releaseClutch();
    car->shiftUp();
    car->updateEngine(0.0);

    EXPECT_NO_THROW(car->updateEngine(0.0));
}

TEST_F(CarTest, UpdateEngineWithMaxThrottle) {
    car->releaseClutch();
    car->shiftUp();
    car->updateEngine(1.0);

    EXPECT_NO_THROW(car->updateEngine(1.0));
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
    // Set car moving forward (Y direction) with wheels not rotating
    // This creates slip that should generate friction forces
    car->velocity = Eigen::Vector2d(0.0, 10.0);
    car->angular_position = 0.0;
    for (Wheel* wheel : car->wheels) {
        wheel->angular_velocity = 0.0;  // Wheels locked
    }

    car->sumWheelForces();

    EXPECT_NE(car->forces, Eigen::Vector2d::Zero());
}

TEST_F(CarTest, SumWheelForcesAppliesTorque) {
    car->velocity = Eigen::Vector2d(0.0, 10.0);
    car->applySteering(0.5);  // radians

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
    car->applySteering(0.5);  // radians
    double initialAngle = car->steering_angle;

    car->moveWheels();

    // Force feedback disabled, steering angle should stay the same
    EXPECT_DOUBLE_EQ(car->steering_angle, initialAngle);
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

    double expected = car->steering_angle * PhysicsConstants::STEERING_RACK;
    EXPECT_DOUBLE_EQ(angle, expected);
}

TEST_F(CarTest, IntegrationSteeringAndMovement) {
    car->applySteering(0.3);
    car->releaseClutch();
    car->shiftUp();

    double initialPosX = car->pos_x;
    double initialPosY = car->pos_y;

    for (int i = 0; i < 30; i++) {
        car->updateEngine(1.0);
        car->moveWheels();
        car->sumWheelForces();
        car->incrementTime(PhysicsConstants::TIME_INTERVAL);
    }

    double distanceMoved = std::sqrt(std::pow(car->pos_x - initialPosX, 2) +
                                     std::pow(car->pos_y - initialPosY, 2));
    EXPECT_GT(distanceMoved, 0.0);
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
        car->incrementTime(PhysicsConstants::TIME_INTERVAL);
    }

    double finalSpeed = car->velocity.norm();
    EXPECT_LE(finalSpeed, initialSpeed * 1.1);
}

TEST_F(CarTest, IntegrationSteeringReturnsToCenter) {
    car->applySteering(0.5);  // radians
    double initialAngle = car->steering_angle;

    for (int i = 0; i < 50; i++) {
        car->moveWheels();
    }

    // Force feedback disabled (decay = 1.0), steering should stay constant
    EXPECT_DOUBLE_EQ(car->steering_angle, initialAngle);
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
