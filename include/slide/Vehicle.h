#pragma once

#include <array>

#include "slide/Config.h"
#include "slide/DriverAids.h"
#include "slide/Drivetrain.h"
#include "slide/Engine.h"
#include "slide/Math.h"
#include "slide/Suspension.h"
#include "slide/Terrain.h"
#include "slide/Tire.h"

namespace slide {

struct DriverInput {
    double throttle = 0.0;  // 0..1
    double brake = 0.0;     // 0..1
    double steer = 0.0;     // -1..1, positive = left
    double clutch = 0.0;    // 0..1, 1 = pedal to the floor
    bool handbrake = false;
};

struct Wheel {
    // Fixed configuration.
    Vec2 position;            // contact patch in body frame [m]
    double radius = 0.0;
    double inertia = 0.0;
    double maxBrakeTorque = 0.0;

    // State.
    double omega = 0.0;       // spin rate [rad/s]
    double spinAngle = 0.0;
    double steerAngle = 0.0;  // [rad], + = left
    double brakeTorque = 0.0; // after hydraulic lag [N m]
    TireContact contact;
    double roadHeight = 0.0;

    // Last step outputs.
    TireOutput tire;
    SurfaceSample surface;
    Vec2 groundVelocity;      // contact patch velocity, wheel frame [m/s]
    Vec2 force;               // tire force, body frame [N]
    double driveTorque = 0.0;
};

// Full vehicle: planar rigid body (x, y, yaw) carrying a sprung body (heave,
// pitch, roll), four spinning wheels with transient tires, and an engine and
// drivetrain coupled through a friction clutch. Advanced with a fixed step;
// identical inputs always produce identical outputs.
class Vehicle {
public:
    explicit Vehicle(const VehicleParams& params = {}, const Terrain* terrain = nullptr);

    void reset(Vec2 position, double heading, double speed = 0.0);
    void step(const DriverInput& input, double dt);

    // Pose and motion. World frame is x east, y north; heading is CCW from +x.
    Vec2 position() const { return position_; }
    double heading() const { return heading_; }
    Vec2 velocity() const { return velocity_; }
    Vec2 localVelocity() const { return Rot2(heading_).applyInverse(velocity_); }
    Vec2 localAcceleration() const { return localAcceleration_; }  // [m/s^2]
    double yawRate() const { return yawRate_; }
    double speed() const { return velocity_.length(); }
    double sideslip() const;
    double time() const { return time_; }
    double steerAngle() const { return steerAngle_; }  // bicycle-equivalent [rad]
    double maxSteerAngle() const;  // full-lock angle at the current speed [rad]
    Vec2 worldPoint(Vec2 bodyPoint) const { return position_ + Rot2(heading_).apply(bodyPoint); }

    const VehicleParams& params() const { return params_; }
    const std::array<Wheel, kCornerCount>& wheels() const { return wheels_; }
    const Wheel& wheel(int corner) const { return wheels_[static_cast<size_t>(corner)]; }
    const TireModel& tireModel(int corner) const { return tireModels_[isFront(corner) ? 0 : 1]; }
    const Suspension& suspension() const { return suspension_; }
    const Engine& engine() const { return engine_; }
    Engine& engine() { return engine_; }
    const Drivetrain& drivetrain() const { return drivetrain_; }
    Drivetrain& drivetrain() { return drivetrain_; }
    const DriverAids& aids() const { return aids_; }
    DriverAids& aids() { return aids_; }
    const DriverInput& lastInput() const { return input_; }

    void setTerrain(const Terrain* terrain) { terrain_ = terrain; }
    const Terrain& terrain() const { return terrain_ ? *terrain_ : flat_; }
    double drivenWheelOmega() const;
    double dragForce() const { return dragForce_; }
    double downforce() const { return frontDownforce_ + rearDownforce_; }

private:
    void updateSteering(double input, double dt);

    VehicleParams params_;
    std::array<TireModel, 2> tireModels_;  // front, rear
    Suspension suspension_;
    Engine engine_;
    Drivetrain drivetrain_;
    DriverAids aids_;
    FlatTerrain flat_;
    const Terrain* terrain_;  // not owned; flat ground when null

    std::array<Wheel, kCornerCount> wheels_;
    Vec2 position_;
    Vec2 velocity_;
    double heading_ = 0.0;
    double yawRate_ = 0.0;
    Vec2 localAcceleration_;
    double steerInput_ = 0.0;
    double steerAngle_ = 0.0;
    double frontLateral_ = 0.0;
    double rearLateral_ = 0.0;
    double frontDownforce_ = 0.0;
    double rearDownforce_ = 0.0;
    double dragForce_ = 0.0;
    double time_ = 0.0;
    bool firstStep_ = true;
    DriverInput input_;
};

}  // namespace slide
