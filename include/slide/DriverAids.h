#pragma once

#include <algorithm>
#include <array>

#include "slide/Config.h"
#include "slide/Suspension.h"

namespace slide {

// Wheel-speed and inertial measurements the control unit works from.
struct AidSensors {
    std::array<double, kCornerCount> wheelSpeed{};    // omega * R [m/s]
    std::array<double, kCornerCount> groundSpeed{};   // contact patch, wheel frame [m/s]
    double forwardSpeed = 0.0;  // body frame [m/s]
    double lateralSpeed = 0.0;
    double yawRate = 0.0;       // [rad/s]
    double steerAngle = 0.0;    // equivalent bicycle-model road wheel angle [rad]
    double wheelbase = 0.0;
    double brakePedal = 0.0;
    double throttle = 0.0;
};

// ABS, traction control and yaw stability control.
class DriverAids {
public:
    explicit DriverAids(const DriverAidParams& params) : p_(params) {}

    void update(const AidSensors& s, double dt);

    DriverAidParams& params() { return p_; }
    const DriverAidParams& params() const { return p_; }

    // Fraction of driver brake pressure each wheel receives (ABS).
    double brakeModulation(int corner) const { return absModulation_[static_cast<size_t>(corner)]; }
    // Extra brake torque requested by stability control [N m].
    double stabilityBrake(int corner) const { return escBrake_[static_cast<size_t>(corner)]; }
    // Engine torque reduction, 0..1.
    double torqueCut() const { return std::max(tcsCut_, escCut_); }

    bool absActive() const { return absActive_; }
    bool tcsActive() const { return tcsCut_ > 0.02; }
    bool escActive() const { return escActive_; }
    double yawRateTarget() const { return yawRateTarget_; }

private:
    void updateAbs(const AidSensors& s, double dt);
    void updateTcs(const AidSensors& s, double dt);
    void updateEsc(const AidSensors& s);

    DriverAidParams p_;
    std::array<double, kCornerCount> absPressure_{1.0, 1.0, 1.0, 1.0};
    std::array<double, kCornerCount> absModulation_{1.0, 1.0, 1.0, 1.0};
    std::array<double, kCornerCount> escBrake_{};
    double tcsIntegral_ = 0.0;
    double tcsCut_ = 0.0;
    double escCut_ = 0.0;
    double yawRateTarget_ = 0.0;
    bool absActive_ = false;
    bool escActive_ = false;
};

}  // namespace slide
