#include "slide/DriverAids.h"

#include <algorithm>
#include <cmath>

namespace slide {
namespace {

double slipRatio(double wheelSpeed, double groundSpeed) {
    return (wheelSpeed - groundSpeed) / std::max(std::abs(groundSpeed), 0.5);
}

}  // namespace

void DriverAids::update(const AidSensors& s, double dt) {
    updateAbs(s, dt);
    updateTcs(s, dt);
    updateEsc(s);
}

// PI pressure modulator per wheel: the integral bleeds pressure off while the
// wheel is past the slip target and rebuilds it below, rate-limited like real
// valve hardware; the proportional term damps the limit cycle that the brake
// hydraulics would otherwise set up.
void DriverAids::updateAbs(const AidSensors& s, double dt) {
    absActive_ = false;
    const bool armed = p_.absEnabled && s.brakePedal > 0.01;
    for (int i = 0; i < kCornerCount; ++i) {
        double& pressure = absPressure_[static_cast<size_t>(i)];
        double& mod = absModulation_[static_cast<size_t>(i)];
        const double ground = s.groundSpeed[static_cast<size_t>(i)];
        if (!armed || std::abs(ground) < p_.absMinSpeed) {
            pressure = mod = 1.0;
            continue;
        }
        // Normalize so braking slip is negative in either travel direction.
        const double slip = slipRatio(s.wheelSpeed[static_cast<size_t>(i)], ground) * sign(ground);
        const double excess = -slip - p_.absSlipTarget;
        const double rate = std::clamp(-p_.absKi * excess, -p_.absReleaseRate, p_.absApplyRate);
        pressure = std::clamp(pressure + rate * dt, 0.0, 1.0);
        mod = std::clamp(pressure - p_.absKp * excess, 0.0, 1.0);
        if (mod < 0.999) absActive_ = true;
    }
}

// PI controller on the worst driven-wheel slip, acting on engine torque. Near
// standstill slip ratio is meaningless, so it switches to slip speed.
void DriverAids::updateTcs(const AidSensors& s, double dt) {
    if (!p_.tcsEnabled || s.throttle < 0.05) {
        tcsIntegral_ = 0.0;
        tcsCut_ = 0.0;
        return;
    }
    double excess = -1.0;
    for (int i : {kRearLeft, kRearRight}) {
        const double wheel = s.wheelSpeed[static_cast<size_t>(i)];
        const double ground = s.groundSpeed[static_cast<size_t>(i)];
        const double slipSpeed = (wheel - ground) * sign(wheel);
        const double e = std::abs(ground) > 3.0
                             ? slipRatio(wheel, ground) * sign(ground) - p_.tcsSlipTarget
                             : (slipSpeed - p_.tcsSlipSpeed) / 3.0;
        excess = std::max(excess, e);
    }
    tcsIntegral_ = std::clamp(tcsIntegral_ + p_.tcsKi * excess * dt, 0.0, p_.tcsMaxCut);
    tcsCut_ = std::clamp(p_.tcsKp * excess + tcsIntegral_, 0.0, p_.tcsMaxCut);
}

// Compares measured yaw rate with a steady-state bicycle-model reference and
// brakes single wheels to create a correcting yaw moment.
void DriverAids::updateEsc(const AidSensors& s) {
    escBrake_.fill(0.0);
    escCut_ = 0.0;
    escActive_ = false;
    const double vx = s.forwardSpeed;
    yawRateTarget_ = 0.0;
    if (vx > 0.1) {
        yawRateTarget_ =
            vx * s.steerAngle / (s.wheelbase + p_.escUndersteerGradient * vx * vx);
        const double limit = 0.85 * kGravity / vx;  // what the tires can sustain
        yawRateTarget_ = std::clamp(yawRateTarget_, -limit, limit);
    }
    if (!p_.escEnabled || vx < p_.escMinSpeed) return;

    const double direction = std::abs(yawRateTarget_) > 0.02 ? sign(yawRateTarget_) : sign(s.yawRate);
    if (direction == 0.0) return;
    // Positive = rotating more than the reference (oversteer).
    double error = (s.yawRate - yawRateTarget_) * direction;
    const double sideslip = std::atan2(s.lateralSpeed, vx);
    error = std::max(error, std::abs(sideslip) - p_.escSideslipLimit);

    const double deadband = p_.escDeadband + 0.1 * std::abs(yawRateTarget_);
    if (std::abs(error) < deadband) return;
    const double torque = std::min(p_.escMaxTorque, p_.escGain * (std::abs(error) - deadband));
    const bool turningLeft = direction > 0.0;
    if (error > 0.0) {
        // Oversteer: brake the outside front wheel and trim engine torque.
        escBrake_[turningLeft ? kFrontRight : kFrontLeft] = torque;
        escCut_ = std::min(0.6, 2.0 * (error - deadband));
    } else {
        // Understeer: brake the inside rear wheel.
        escBrake_[turningLeft ? kRearLeft : kRearRight] = torque;
    }
    escActive_ = true;
}

}  // namespace slide
