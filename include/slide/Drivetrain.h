#pragma once

#include "slide/Config.h"

namespace slide {

class Engine;

// Clutch, gearbox, shift sequencing and a clutch-pack LSD feeding the rear
// axle. The clutch is solved as a velocity constraint: each step it applies the
// torque that would bring slip to zero, clamped to its friction capacity. That
// one rule covers lock-up, slipping launches and gear changes without separate
// state machines or stiff springs.
class Drivetrain {
public:
    enum class Mode { Automatic, Manual };

    struct AxleTorque {
        double left = 0.0;   // drive torque at each wheel [N m]
        double right = 0.0;
    };

    explicit Drivetrain(const DrivetrainParams& params);

    // Gear numbering: -1 reverse, 0 neutral, 1..N forward.
    int gear() const { return gear_; }
    bool shifting() const { return shiftTimer_ > 0.0; }
    Mode mode() const { return mode_; }
    void setMode(Mode mode) { mode_ = mode; }
    bool requestGear(int gear, double drivenOmega);
    bool requestShift(int delta, double drivenOmega) {
        return requestGear((shifting() ? pendingGear_ : gear_) + delta, drivenOmega);
    }
    void setClutchPedal(double pedal) { clutchPedal_ = pedal; }  // 1 = fully pressed
    // Puts the box straight into a gear with the clutch closed (test setup).
    void engage(int gear);

    // Overall engine:wheel ratio for a gear, signed (negative in reverse).
    double ratioFor(int gear) const;
    double ratio() const { return ratioFor(gear_); }

    // Runs the automatic clutch and shift logic for this step.
    void updateControl(const Engine& engine, double pedal, double brake, double drivenOmega,
                       double dt);

    // Solves clutch and differential torques for this step. `externalLeft` and
    // `externalRight` are the non-drivetrain torques on the driven wheels.
    AxleTorque solve(const Engine& engine, double omegaLeft, double omegaRight,
                     double externalLeft, double externalRight, double wheelInertia,
                     double dt);

    // Torque modulation requested by the shift sequence: an ignition cut on
    // upshifts and a throttle blip to rev-match downshifts.
    double shiftTorqueCut() const { return shiftCut_; }
    double shiftThrottleBlip() const { return shiftBlip_; }

    double clutchEngagement() const { return engagement_; }
    double clutchCapacity() const;
    double clutchTorque() const { return clutchTorque_; }
    double clutchSlip() const { return clutchSlip_; }  // engine - gearbox input [rad/s]
    bool clutchLocked() const { return clutchLocked_; }
    double lsdTorque() const { return lsdTorque_; }
    const DrivetrainParams& params() const { return p_; }

private:
    DrivetrainParams p_;
    Mode mode_ = Mode::Automatic;
    int gear_ = 1;
    int pendingGear_ = 1;
    double shiftTimer_ = 0.0;
    double sinceShift_ = 1.0;
    bool upshifting_ = false;
    double clutchPedal_ = 0.0;
    double engagement_ = 0.0;
    bool autoClutchClosed_ = false;
    bool clutchLocked_ = false;
    double clutchTorque_ = 0.0;
    double clutchSlip_ = 0.0;
    double lsdTorque_ = 0.0;
    double shiftCut_ = 0.0;
    double shiftBlip_ = 0.0;
};

}  // namespace slide
