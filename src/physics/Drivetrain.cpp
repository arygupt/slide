#include "slide/Drivetrain.h"

#include <algorithm>
#include <cmath>

#include "slide/Engine.h"

namespace slide {
namespace {

constexpr double kShiftSwapPoint = 0.4;    // fraction of the shift when the ratio swaps
constexpr double kMinTimeBetweenShifts = 0.6;   // upshifts, to avoid hunting
constexpr double kMinTimeBetweenDownshifts = 0.25;
constexpr double kSyncedSlip = 15.0;       // [rad/s]
constexpr double kMaxSyncTime = 0.4;       // [s]
constexpr double kLockSlip = 8.0;          // [rad/s]
constexpr double kReverseGuardOmega = 2.0; // driven wheel speed allowed for R <-> 1 [rad/s]

}  // namespace

Drivetrain::Drivetrain(const DrivetrainParams& params) : p_(params) {}

double Drivetrain::ratioFor(int gear) const {
    if (gear == 0) return 0.0;
    if (gear < 0) return -p_.reverseRatio * p_.finalDrive;
    return p_.gearRatios[static_cast<size_t>(gear - 1)] * p_.finalDrive;
}

double Drivetrain::clutchCapacity() const {
    return p_.clutchMaxTorque * engagement_ * engagement_;
}

void Drivetrain::engage(int gear) {
    gear_ = pendingGear_ = std::clamp(gear, -1, DrivetrainParams::kGearCount);
    shiftTimer_ = 0.0;
    engagement_ = 1.0;
    autoClutchClosed_ = gear_ != 0;
}

bool Drivetrain::requestGear(int gear, double drivenOmega) {
    gear = std::clamp(gear, -1, DrivetrainParams::kGearCount);
    const int current = shifting() ? pendingGear_ : gear_;
    if (gear == current) return false;

    // Direction changes only near standstill; downshifts only if the engine
    // would survive the resulting speed.
    const bool reversing = (gear < 0 && current > 0) || (gear > 0 && current < 0);
    if (reversing && std::abs(drivenOmega) > kReverseGuardOmega) return false;
    if (gear > 0 && gear < current) {
        const double newRpm = std::abs(ratioFor(gear) * drivenOmega) * kRadToRpm;
        if (newRpm > p_.overRevGuardRpm) return false;
    }

    upshifting_ = current > 0 && gear > current;
    pendingGear_ = gear;
    shiftTimer_ = p_.shiftTime;
    return true;
}

void Drivetrain::updateControl(const Engine& engine, double pedal, double brake,
                               double drivenOmega, double dt) {
    const double rpm = engine.rpm();
    shiftCut_ = 0.0;
    shiftBlip_ = 0.0;
    sinceShift_ += dt;

    double target = 1.0;
    double engageRate = p_.clutchEngageRate;
    if (shifting()) {
        const double progress = 1.0 - shiftTimer_ / p_.shiftTime;
        if (progress >= kShiftSwapPoint) gear_ = pendingGear_;
        if (!upshifting_ && pendingGear_ > 0) {
            const double matchRpm = std::abs(ratioFor(pendingGear_) * drivenOmega) * kRadToRpm;
            if (rpm < matchRpm) shiftBlip_ = 0.7;
        }
        if (progress < kShiftSwapPoint) target = 0.0;
        engageRate = 1.0 / ((1.0 - kShiftSwapPoint) * p_.shiftTime);
        shiftTimer_ -= dt;
        if (shiftTimer_ <= 0.0) {
            shiftTimer_ = 0.0;
            sinceShift_ = 0.0;
        }
    }
    // Upshifts cut ignition until the clutch has pulled the engine down to the
    // new gear's speed, so the engine does not flare while the clutch is open.
    if (upshifting_) {
        if (shifting() || std::abs(clutchSlip_) > kSyncedSlip) shiftCut_ = 1.0;
        if (!shifting() && (std::abs(clutchSlip_) <= kSyncedSlip || sinceShift_ > kMaxSyncTime)) {
            upshifting_ = false;
        }
    }

    // Automatic clutch: modulated by engine speed during launch (like a
    // centrifugal clutch) and held closed once slip is gone.
    if (gear_ == 0 || rpm < p_.autoClutchReleaseRpm) autoClutchClosed_ = false;
    if (!autoClutchClosed_ && gear_ != 0 && !shifting() && engagement_ > 0.25 &&
        std::abs(clutchSlip_) < kLockSlip && rpm > p_.autoClutchStartRpm) {
        autoClutchClosed_ = true;
    }
    if (target > 0.0 && !autoClutchClosed_) {
        target = smoothstep(p_.autoClutchStartRpm, p_.autoClutchFullRpm, rpm);
    }
    target = std::min(target, 1.0 - std::clamp(clutchPedal_, 0.0, 1.0));
    const double rate = target > engagement_ ? engageRate : p_.clutchReleaseRate;
    engagement_ = approach(engagement_, target, rate * dt);

    if (mode_ != Mode::Automatic || gear_ < 1 || shifting()) return;

    // Rolling to a stop with the clutch open: drop straight to first.
    if (!autoClutchClosed_) {
        if (gear_ > 1 && clutchPedal_ < 0.5) requestGear(1, drivenOmega);
        return;
    }

    // Shift schedule, evaluated on driveline speed so clutch slip and wheel
    // spin during a launch do not trigger early upshifts.
    const double drivelineRpm = std::abs(ratio() * drivenOmega) * kRadToRpm;
    const double up = lerp(p_.upshiftRpmLight, p_.upshiftRpmFull, pedal);
    // Braking raises the downshift point too (sport-mode behavior), so the
    // car leaves a corner in a gear that pulls.
    const double down =
        lerp(p_.downshiftRpmLight, p_.downshiftRpmFull, std::max(pedal, 0.85 * brake));
    if (drivelineRpm > up && gear_ < DrivetrainParams::kGearCount &&
        sinceShift_ > kMinTimeBetweenShifts) {
        requestGear(gear_ + 1, drivenOmega);
    } else if (gear_ > 1 && drivelineRpm < down && sinceShift_ > kMinTimeBetweenDownshifts) {
        // Skip gears when far below the schedule, e.g. under hard braking.
        int lower = gear_ - 1;
        const auto rpmIn = [&](int g) { return std::abs(ratioFor(g) * drivenOmega) * kRadToRpm; };
        while (lower > 1 && rpmIn(lower) < down) --lower;
        if (rpmIn(lower) < 0.92 * up) requestGear(lower, drivenOmega);
    }
}

Drivetrain::AxleTorque Drivetrain::solve(const Engine& engine, double omegaLeft,
                                         double omegaRight, double externalLeft,
                                         double externalRight, double wheelInertia,
                                         double dt) {
    const double G = ratio();
    const double capacity = clutchCapacity();
    double drive = 0.0;

    if (G == 0.0 || capacity <= 0.0) {
        clutchTorque_ = 0.0;
        clutchLocked_ = false;
        clutchSlip_ = G == 0.0 ? 0.0 : engine.omega() - G * 0.5 * (omegaLeft + omegaRight);
    } else {
        // Clutch torque that makes engine speed equal gearbox input speed at
        // the end of the step:
        //   w_e + dt (T_e - T_c)/I_e = G (w_d + dt (eta G T_c + tau_L + tau_R)/(2 I_w))
        const double eta = clutchTorque_ * engine.omega() >= 0.0 ? p_.efficiency
                                                                 : 1.0 / p_.efficiency;
        const double Ie = engine.inertia();
        const double Iw = wheelInertia;
        clutchSlip_ = engine.omega() - G * 0.5 * (omegaLeft + omegaRight);
        const double numerator = clutchSlip_ / dt + engine.torque() / Ie -
                                 G * (externalLeft + externalRight) / (2.0 * Iw);
        const double lockTorque = numerator / (1.0 / Ie + eta * G * G / (2.0 * Iw));
        clutchLocked_ = std::abs(lockTorque) <= capacity;
        clutchTorque_ = std::clamp(lockTorque, -capacity, capacity);
        drive = eta * G * clutchTorque_;
    }

    // Limited-slip differential: viscous coupling capped by preload plus a
    // share of input torque, moving torque from the faster wheel to the slower.
    const double limit = p_.lsdPreload + p_.lsdPowerLock * std::abs(drive);
    lsdTorque_ = std::clamp(p_.lsdViscous * (omegaLeft - omegaRight), -limit, limit);
    return {0.5 * (drive - lsdTorque_), 0.5 * (drive + lsdTorque_)};
}

}  // namespace slide
