#include "slide/Engine.h"

#include <algorithm>
#include <cmath>

namespace slide {
namespace {

constexpr double kGasConstantAir = 287.05;  // [J/(kg K)]
constexpr double kGamma = 1.4;
constexpr double kCriticalPressureRatio = 0.5283;
constexpr double kOverrunCutRpm = 1600.0;
constexpr double kOverrunResumeRpm = 1300.0;
constexpr double kRunningRpm = 550.0;  // starter hands over above this
constexpr double kIdleControlBand = 400.0;  // [rpm] above idle where the idle PI acts

// Compressible flow function for an isentropic orifice. Above pr = 0.98 the
// curve is linearized so its slope stays finite as the throttle opens wide.
double flowFunction(double pressureRatio) {
    const auto subsonic = [](double pr) {
        const double t = std::pow(pr, 2.0 / kGamma) - std::pow(pr, (kGamma + 1.0) / kGamma);
        return std::sqrt(std::max(0.0, 2.0 * kGamma / (kGamma - 1.0) * t));
    };
    const double pr = std::clamp(pressureRatio, 0.0, 1.0);
    if (pr <= kCriticalPressureRatio) return subsonic(kCriticalPressureRatio);
    constexpr double kLinearStart = 0.98;
    if (pr > kLinearStart) return subsonic(kLinearStart) * (1.0 - pr) / (1.0 - kLinearStart);
    return subsonic(pr);
}

}  // namespace

Engine::Engine(const EngineParams& params)
    : p_(params),
      omega_(params.idleRpm * kRpmToRad),
      plateAngle_(params.throttleClosedAngle),
      manifoldPressure_(0.35 * params.ambientPressure),
      idleIntegral_(0.3 * params.idleValveMaxArea) {}

double Engine::volumetricEfficiency(double rpm) const {
    if (rpm < p_.vePeakRpm) {
        const double t = (p_.vePeakRpm - rpm) / p_.vePeakRpm;
        return p_.vePeak - p_.veLowDrop * t * t;
    }
    const double t = (rpm - p_.vePeakRpm) / (p_.limiterRpm - p_.vePeakRpm);
    return std::max(0.3, p_.vePeak - p_.veHighDrop * t * t);
}

double Engine::frictionTorque(double rpm) const {
    const double krpm = rpm / 1000.0;
    const double fmep = p_.fmepA + p_.fmepB * krpm + p_.fmepC * krpm * krpm;
    return fmep * p_.displacement / (4.0 * kPi);
}

double Engine::throttleArea(double plateAngle) const {
    const double bore = 0.25 * kPi * p_.throttleDiameter * p_.throttleDiameter;
    const double open = 1.0 - std::cos(plateAngle) / std::cos(p_.throttleClosedAngle);
    const double dashpot = overrunCut_ ? p_.overrunAirArea : 0.0;
    return std::max(0.0, bore * open) + idleArea_ + p_.leakArea + dashpot;
}

double Engine::throttleFlow(double manifoldPressure) const {
    const double p0 = p_.ambientPressure;
    return p_.throttleDischarge * throttleArea(plateAngle_) * p0 /
           std::sqrt(kGasConstantAir * p_.ambientTemperature) *
           flowFunction(manifoldPressure / p0);
}

double Engine::cylinderFlow(double manifoldPressure) const {
    // Four-stroke: one intake event per cylinder every two revolutions.
    const double density = manifoldPressure / (kGasConstantAir * p_.ambientTemperature);
    return volumetricEfficiency(rpm()) * density * p_.displacement * omega_ / (4.0 * kPi);
}

void Engine::updateIdleValve(double pedal, double dt) {
    if (!running_) {
        idleIntegral_ = 0.0;
        idleArea_ = 0.0;
        return;
    }
    // The PI loop only governs near idle; elsewhere (driving, overrun) the
    // valve holds its learned position, like an ECU's idle-air adaptation.
    const double error = p_.idleRpm - rpm();
    if (pedal < 0.02 && rpm() < p_.idleRpm + kIdleControlBand) {
        idleIntegral_ = std::clamp(idleIntegral_ + p_.idleKi * error * dt, 0.0, p_.idleValveMaxArea);
        idleArea_ = std::clamp(p_.idleKp * error + idleIntegral_, 0.0, p_.idleValveMaxArea);
    } else {
        idleArea_ = idleIntegral_;
    }
}

void Engine::update(double pedal, double torqueCut, double dt) {
    pedal = std::clamp(pedal, 0.0, 1.0);
    const double r = rpm();

    // Drive-by-wire throttle body: first-order actuator toward the target angle.
    const double target = lerp(p_.throttleClosedAngle, 0.5 * kPi, pedal);
    plateAngle_ += (target - plateAngle_) * (1.0 - std::exp(-dt / p_.throttleTimeConstant));
    updateIdleValve(pedal, dt);

    // Manifold filling, dp/dt = RT/V (mdot_in - mdot_out). The flow balance is
    // stiff near wide-open throttle, so take a linearized implicit Euler step.
    const double rtOverV = kGasConstantAir * p_.ambientTemperature / p_.manifoldVolume;
    const auto rate = [&](double p) { return rtOverV * (throttleFlow(p) - cylinderFlow(p)); };
    const double h = 25.0;
    const double f = rate(manifoldPressure_);
    const double dfdp = (rate(manifoldPressure_ + h) - rate(manifoldPressure_ - h)) / (2.0 * h);
    const double step = dt * f / std::max(1.0, 1.0 - dt * dfdp);
    manifoldPressure_ = std::clamp(manifoldPressure_ + step, 500.0, p_.ambientPressure);

    // Fuel cut-offs: rev limiter and deceleration fuel cut-off on overrun.
    if (r > p_.limiterRpm) limiterCut_ = true;
    else if (r < p_.limiterRpm - p_.limiterHysteresis) limiterCut_ = false;
    if (pedal < 0.01 && r > kOverrunCutRpm) overrunCut_ = true;
    else if (pedal >= 0.01 || r < kOverrunResumeRpm) overrunCut_ = false;
    const bool fueled = running_ && !limiterCut_ && !overrunCut_;

    const double ve = volumetricEfficiency(r);
    const double airflow = cylinderFlow(manifoldPressure_);
    const double loadFraction = manifoldPressure_ / p_.ambientPressure;
    const double efficiency =
        p_.peakIndicatedEfficiency * lerp(p_.partLoadEfficiency, 1.0, loadFraction);
    const double density = manifoldPressure_ / (kGasConstantAir * p_.ambientTemperature);

    // Indicated torque from the fuel burned per cycle; air-limited, so it is
    // computed at stoichiometric even when the ECU is enriching.
    double indicated = 0.0;
    if (fueled) {
        indicated = efficiency * p_.fuelLhv * ve * density * p_.displacement /
                    (4.0 * kPi * p_.stoichAfr);
        indicated *= 1.0 - std::clamp(torqueCut, 0.0, 1.0);
    }

    // Losses oppose rotation and fade to zero as the crank stops.
    const double spin = std::tanh(omega_ / 5.0);
    const double pumping = (p_.exhaustPressureRatio * p_.ambientPressure - manifoldPressure_) *
                           p_.displacement / (4.0 * kPi) * spin;
    const double friction = frictionTorque(r) * spin;
    const double starter = (starter_ && !running_) ? p_.starterTorque : 0.0;

    const double lambda = lerp(1.0, p_.wotLambda, smoothstep(0.7, 0.95, pedal));
    out_.throttleAngle = plateAngle_;
    out_.manifoldPressure = manifoldPressure_;
    out_.throttleAirflow = throttleFlow(manifoldPressure_);
    out_.cylinderAirflow = airflow;
    out_.fuelFlow = fueled ? airflow / (p_.stoichAfr * lambda) : 0.0;
    out_.volumetricEfficiency = ve;
    out_.indicatedTorque = indicated;
    out_.frictionTorque = friction;
    out_.pumpingTorque = pumping;
    out_.netTorque = indicated + starter - pumping - friction;
    out_.idleValveArea = idleArea_;
    out_.fuelCut = running_ && !fueled;
}

void Engine::integrate(double clutchTorque, double dt) {
    omega_ = std::max(0.0, omega_ + (out_.netTorque - clutchTorque) / p_.inertia * dt);
    const double r = rpm();
    if (running_ && r < p_.stallRpm) running_ = false;
    if (!running_ && starter_ && r > kRunningRpm) running_ = true;
}

void Engine::forceSpeed(double omega) {
    omega_ = std::max(0.0, omega);
    if (!running_ && rpm() > kRunningRpm) running_ = true;
}

}  // namespace slide
