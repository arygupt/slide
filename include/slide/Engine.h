#pragma once

#include "slide/Config.h"

namespace slide {

// Mean-value spark-ignition engine. Torque is not read from a curve: it falls
// out of the air path (throttle orifice -> manifold plenum -> cylinders), a
// fuel energy balance, and friction/pumping losses.
class Engine {
public:
    struct Readout {
        double throttleAngle = 0.0;     // plate angle [rad]
        double manifoldPressure = 0.0;  // MAP [Pa]
        double throttleAirflow = 0.0;   // [kg/s]
        double cylinderAirflow = 0.0;   // [kg/s]
        double fuelFlow = 0.0;          // [kg/s]
        double volumetricEfficiency = 0.0;
        double indicatedTorque = 0.0;   // [N m]
        double frictionTorque = 0.0;
        double pumpingTorque = 0.0;
        double netTorque = 0.0;         // brake torque at the crank [N m]
        double idleValveArea = 0.0;     // [m^2]
        bool fuelCut = false;           // limiter or overrun cut-off
    };

    explicit Engine(const EngineParams& params);

    // Updates the throttle actuator, manifold and combustion at the current
    // crank speed. `torqueCut` (0..1) is a traction-control spark/fuel cut.
    void update(double pedal, double torqueCut, double dt);

    // Advances crank speed given the torque taken by the clutch.
    void integrate(double clutchTorque, double dt);

    void setStarter(bool engaged) { starter_ = engaged; }
    void forceSpeed(double omega);  // dyno speed control and test setup

    double omega() const { return omega_; }
    double rpm() const { return omega_ * kRadToRpm; }
    double inertia() const { return p_.inertia; }
    bool running() const { return running_; }
    double torque() const { return out_.netTorque; }
    const Readout& readout() const { return out_; }
    const EngineParams& params() const { return p_; }

    double volumetricEfficiency(double rpm) const;
    double frictionTorque(double rpm) const;

private:
    double throttleArea(double plateAngle) const;
    double throttleFlow(double manifoldPressure) const;
    double cylinderFlow(double manifoldPressure) const;
    void updateIdleValve(double pedal, double dt);

    EngineParams p_;
    double omega_;
    double plateAngle_;
    double manifoldPressure_;
    double idleIntegral_;
    double idleArea_ = 0.0;
    bool running_ = true;
    bool starter_ = false;
    bool limiterCut_ = false;
    bool overrunCut_ = false;
    Readout out_;
};

}  // namespace slide
