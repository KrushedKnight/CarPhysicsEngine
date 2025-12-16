#include <gtest/gtest.h>
#include "core/RigidBody.h"
#include "config/PhysicsConstants.h"
#include "config/RenderingConstants.h"
#include <Eigen/Dense>

class RigidBodyTest : public ::testing::Test {
protected:
    RigidBody* body;

    void SetUp() override {
        body = new RigidBody();
    }

    void TearDown() override {
        delete body;
    }
};

TEST_F(RigidBodyTest, ConstructorInitializesCorrectly) {
    EXPECT_EQ(body->velocity, Eigen::Vector2d::Zero());
    EXPECT_EQ(body->acceleration, Eigen::Vector2d::Zero());
    EXPECT_EQ(body->forces, Eigen::Vector2d::Zero());
    EXPECT_DOUBLE_EQ(body->angular_position, 0.0);
    EXPECT_DOUBLE_EQ(body->angular_velocity, 0.0);
    EXPECT_DOUBLE_EQ(body->angular_acceleration, 0.0);
    EXPECT_DOUBLE_EQ(body->angular_torque, 0.0);
    EXPECT_DOUBLE_EQ(body->mass, PhysicsConstants::CAR_MASS);
    EXPECT_DOUBLE_EQ(body->moment_of_inertia, RenderingConstants::CAR_MOMENT_OF_INERTIA);
}

TEST_F(RigidBodyTest, GetPositionReturnsFlooredValues) {
    body->pos_x = 10.7;
    body->pos_y = 20.3;
    EXPECT_EQ(body->getPositionX(), 10);
    EXPECT_EQ(body->getPositionY(), 20);
}

TEST_F(RigidBodyTest, GetPositionHandlesNegativeValues) {
    body->pos_x = -5.9;
    body->pos_y = -10.1;
    // std::floor rounds toward negative infinity
    EXPECT_EQ(body->getPositionX(), -6);  // floor(-5.9) = -6
    EXPECT_EQ(body->getPositionY(), -11); // floor(-10.1) = -11
}

TEST_F(RigidBodyTest, AddForceAccumulatesForces) {
    Eigen::Vector2d force1(10.0, 5.0);
    Eigen::Vector2d force2(3.0, 7.0);

    body->addForce(force1);
    EXPECT_EQ(body->forces, force1);

    body->addForce(force2);
    Eigen::Vector2d expected(13.0, 12.0);
    EXPECT_EQ(body->forces, expected);
}

TEST_F(RigidBodyTest, AddForceHandlesNegativeForces) {
    Eigen::Vector2d force(-10.0, -5.0);
    body->addForce(force);
    EXPECT_EQ(body->forces, force);
}

TEST_F(RigidBodyTest, AddTorqueAccumulatesTorques) {
    body->addTorque(10.0);
    EXPECT_DOUBLE_EQ(body->angular_torque, 10.0);

    body->addTorque(5.0);
    EXPECT_DOUBLE_EQ(body->angular_torque, 15.0);
}

TEST_F(RigidBodyTest, AddTorqueHandlesNegativeTorques) {
    body->addTorque(-10.0);
    EXPECT_DOUBLE_EQ(body->angular_torque, -10.0);
}

TEST_F(RigidBodyTest, ClearForcesSetsToZero) {
    body->addForce(Eigen::Vector2d(10.0, 5.0));
    body->clearForces();
    EXPECT_EQ(body->forces, Eigen::Vector2d::Zero());
}

TEST_F(RigidBodyTest, ClearTorquesSetsToZero) {
    body->addTorque(10.0);
    body->clearTorques();
    EXPECT_DOUBLE_EQ(body->angular_torque, 0.0);
}

TEST_F(RigidBodyTest, IncrementTimeUpdatesLinearMotion) {
    body->pos_x = 0.0;
    body->pos_y = 0.0;
    body->velocity = Eigen::Vector2d(10.0, 5.0);
    body->addForce(Eigen::Vector2d(1.0, 0.5));

    double dt = 0.016;
    body->incrementTime(dt);

    EXPECT_GT(body->pos_x, 0.0);
    EXPECT_LT(body->pos_y, 0.0);

    // With air resistance (drag coefficient 0.4), velocity will decrease slightly
    // even with small applied force, since drag = -0.4 * speed * velocity
    // At speed ~11.18 m/s, drag force is ~44.7 N, much larger than applied force (1.0, 0.5) N
    EXPECT_LT(body->velocity.x(), 10.0);
    EXPECT_LT(body->velocity.y(), 5.0);

    EXPECT_EQ(body->forces, Eigen::Vector2d::Zero());
}

TEST_F(RigidBodyTest, IncrementTimeUpdatesAngularMotion) {
    body->angular_position = 0.0;
    body->angular_velocity = 10.0;
    body->addTorque(5.0);

    double dt = 0.016;
    body->incrementTime(dt);

    EXPECT_GT(body->angular_position, 0.0);

    EXPECT_GT(body->angular_velocity, 10.0);

    EXPECT_DOUBLE_EQ(body->angular_torque, 0.0);
}

TEST_F(RigidBodyTest, IncrementTimeWithZeroMassDoesNotCrash) {
    body->mass = 0.0;
    body->addForce(Eigen::Vector2d(10.0, 5.0));

    EXPECT_NO_THROW(body->incrementTime(0.016));
}

TEST_F(RigidBodyTest, IncrementTimeWithZeroMomentOfInertiaDoesNotCrash) {
    body->moment_of_inertia = 0.0;
    body->addTorque(10.0);

    EXPECT_NO_THROW(body->incrementTime(0.016));
}

TEST_F(RigidBodyTest, MultipleTimeStepsAccumulateMotion) {
    body->pos_x = 0.0;
    body->pos_y = 0.0;
    body->velocity = Eigen::Vector2d(0.0, 0.0);

    double dt = 0.016;
    for (int i = 0; i < 10; i++) {
        body->addForce(Eigen::Vector2d(10.0, 0.0));
        body->incrementTime(dt);
    }

    EXPECT_GT(body->pos_x, 0.0);

    EXPECT_GT(body->velocity.x(), 0.0);
}
