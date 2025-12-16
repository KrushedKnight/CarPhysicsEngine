#include <gtest/gtest.h>
#include "vehicle/Wheel.h"
#include "config/PhysicsConstants.h"
#include <Eigen/Dense>
#include <cmath>

class WheelTest : public ::testing::Test {
protected:
    Wheel* wheel;

    void SetUp() override {
        wheel = new Wheel();
    }

    void TearDown() override {
        delete wheel;
    }
};

TEST_F(WheelTest, ConstructorInitializesCorrectly) {
    EXPECT_DOUBLE_EQ(wheel->wheelAngle, 0.0);
    EXPECT_DOUBLE_EQ(wheel->wheelRadius, PhysicsConstants::WHEEL_RADIUS);
    EXPECT_DOUBLE_EQ(wheel->frictionCoefficient, PhysicsConstants::WHEEL_FRICTION);
    EXPECT_DOUBLE_EQ(wheel->normalForce, PhysicsConstants::CAR_WEIGHT / 4.0);
    EXPECT_DOUBLE_EQ(wheel->mass, PhysicsConstants::WHEEL_MASS);
    EXPECT_DOUBLE_EQ(wheel->moment_of_inertia, PhysicsConstants::WHEEL_MOMENT_OF_INERTIA);
}

TEST_F(WheelTest, GetLinearVelocityCalculatesCorrectly) {
    wheel->angular_velocity = 10.0;
    double expected = 10.0 * PhysicsConstants::WHEEL_RADIUS;
    EXPECT_DOUBLE_EQ(wheel->getLinearVelocity(), expected);
}

TEST_F(WheelTest, GetLinearVelocityHandlesNegativeAngularVelocity) {
    wheel->angular_velocity = -5.0;
    double expected = -5.0 * PhysicsConstants::WHEEL_RADIUS;
    EXPECT_DOUBLE_EQ(wheel->getLinearVelocity(), expected);
}

TEST_F(WheelTest, SetLinearVelocityUpdatesAngularVelocity) {
    wheel->setLinearVelocity(5.0);
    double expected = 5.0 / PhysicsConstants::WHEEL_RADIUS;
    EXPECT_DOUBLE_EQ(wheel->angular_velocity, expected);
}

TEST_F(WheelTest, SetLinearVelocityHandlesZero) {
    wheel->angular_velocity = 10.0;
    wheel->setLinearVelocity(0.0);
    EXPECT_DOUBLE_EQ(wheel->angular_velocity, 0.0);
}

TEST_F(WheelTest, CalculateFrictionReturnsZeroForZeroSlip) {
    Eigen::Vector2d carVelocity(0.0, 0.0);
    double carAngularPosition = 0.0;
    wheel->wheelAngle = 0.0;
    wheel->angular_velocity = 0.0;

    Eigen::Vector2d friction = wheel->calculateFriction(carVelocity, carAngularPosition, PhysicsConstants::TIME_INTERVAL);

    EXPECT_EQ(friction, Eigen::Vector2d::Zero());
}

TEST_F(WheelTest, CalculateFrictionWithForwardMotion) {
    Eigen::Vector2d carVelocity(0.0, 10.0);
    double carAngularPosition = 0.0;
    wheel->wheelAngle = 0.0;
    wheel->angular_velocity = 0.0;

    Eigen::Vector2d friction = wheel->calculateFriction(carVelocity, carAngularPosition, PhysicsConstants::TIME_INTERVAL);

    EXPECT_LT(friction.y(), 0.0);
}

TEST_F(WheelTest, CalculateFrictionWithWheelSpinningFaster) {
    Eigen::Vector2d carVelocity(0.0, 5.0);
    double carAngularPosition = 0.0;
    wheel->wheelAngle = 0.0;
    wheel->angular_velocity = 50.0;

    Eigen::Vector2d friction = wheel->calculateFriction(carVelocity, carAngularPosition, PhysicsConstants::TIME_INTERVAL);

    EXPECT_GT(friction.y(), 0.0);
}

TEST_F(WheelTest, CalculateFrictionWithAngledWheel) {
    Eigen::Vector2d carVelocity(0.0, 10.0);
    double carAngularPosition = 0.0;
    wheel->wheelAngle = M_PI / 4.0;
    wheel->angular_velocity = 0.0;

    Eigen::Vector2d friction = wheel->calculateFriction(carVelocity, carAngularPosition, PhysicsConstants::TIME_INTERVAL);

    EXPECT_NE(friction, Eigen::Vector2d::Zero());
}

TEST_F(WheelTest, FrictionIsClampedByMaxFriction) {
    Eigen::Vector2d carVelocity(0.0, 1000.0);
    double carAngularPosition = 0.0;
    wheel->wheelAngle = 0.0;
    wheel->angular_velocity = 0.0;

    Eigen::Vector2d friction = wheel->calculateFriction(carVelocity, carAngularPosition, PhysicsConstants::TIME_INTERVAL);

    double maxFriction = wheel->normalForce * wheel->frictionCoefficient;
    EXPECT_LE(friction.norm(), maxFriction * 1.01);
}

TEST_F(WheelTest, CalculateFrictionAppliesTorqueToWheel) {
    wheel->angular_torque = 0.0;
    Eigen::Vector2d carVelocity(0.0, 10.0);
    double carAngularPosition = 0.0;
    wheel->wheelAngle = 0.0;
    wheel->angular_velocity = 0.0;

    wheel->calculateFriction(carVelocity, carAngularPosition, PhysicsConstants::TIME_INTERVAL);

    EXPECT_NE(wheel->angular_torque, 0.0);
}

TEST_F(WheelTest, IncrementTimeUpdatesWheelRotation) {
    wheel->angular_velocity = 10.0;
    wheel->angular_position = 0.0;
    wheel->addTorque(5.0);

    wheel->incrementTime(PhysicsConstants::TIME_INTERVAL);

    EXPECT_GT(wheel->angular_position, 0.0);

    EXPECT_DOUBLE_EQ(wheel->angular_torque, 0.0);
}

TEST_F(WheelTest, IncrementTimeWithNoTorquePreservesVelocity) {
    wheel->angular_velocity = 10.0;
    double initialVelocity = wheel->angular_velocity;

    wheel->incrementTime(PhysicsConstants::TIME_INTERVAL);

    EXPECT_DOUBLE_EQ(wheel->angular_velocity, initialVelocity);
}

TEST_F(WheelTest, IncrementTimeWithPositiveTorqueIncreasesVelocity) {
    wheel->angular_velocity = 0.0;
    wheel->addTorque(10.0);

    wheel->incrementTime(PhysicsConstants::TIME_INTERVAL);

    EXPECT_GT(wheel->angular_velocity, 0.0);
}

TEST_F(WheelTest, IncrementTimeWithNegativeTorqueDecreasesVelocity) {
    wheel->angular_velocity = 10.0;
    wheel->addTorque(-10.0);

    wheel->incrementTime(PhysicsConstants::TIME_INTERVAL);

    EXPECT_LT(wheel->angular_velocity, 10.0);
}

TEST_F(WheelTest, CalculateFrictionWithDifferentCarOrientations) {
    Eigen::Vector2d carVelocity(10.0, 0.0);
    wheel->wheelAngle = 0.0;
    wheel->angular_velocity = 0.0;

    Eigen::Vector2d friction1 = wheel->calculateFriction(carVelocity, 0.0, PhysicsConstants::TIME_INTERVAL);

    Eigen::Vector2d friction2 = wheel->calculateFriction(carVelocity, M_PI / 2.0, PhysicsConstants::TIME_INTERVAL);

    EXPECT_NE(friction1, friction2);
}

TEST_F(WheelTest, MultipleFrictionCalculationsAccumulateTorque) {
    Eigen::Vector2d carVelocity(0.0, 10.0);
    double carAngularPosition = 0.0;
    wheel->wheelAngle = 0.0;
    wheel->angular_velocity = 0.0;

    wheel->calculateFriction(carVelocity, carAngularPosition, PhysicsConstants::TIME_INTERVAL);
    double torque1 = wheel->angular_torque;

    wheel->calculateFriction(carVelocity, carAngularPosition, PhysicsConstants::TIME_INTERVAL);
    double torque2 = wheel->angular_torque;

    EXPECT_DOUBLE_EQ(torque2, 2.0 * torque1);
}
