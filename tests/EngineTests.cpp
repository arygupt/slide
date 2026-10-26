#include <algorithm>

#include "Check.h"
#include "slide/Engine.h"

using namespace slide;

namespace {

constexpr double kDt = 0.001;

// Steady WOT brake torque with crank speed held by an ideal dyno.
double dynoTorque(double rpm) {
    Engine engine(EngineParams{});
    for (int i = 0; i < 1500; ++i) {
        engine.forceSpeed(rpm * kRpmToRad);
        engine.update(1.0, 0.0, kDt);
    }
    return engine.torque();
}

}  // namespace

TEST(engineIdlesStablyWithNoLoad) {
    Engine engine(EngineParams{});
    double low = 1e9, high = 0.0;
    for (int i = 0; i < 6000; ++i) {
        engine.update(0.0, 0.0, kDt);
        engine.integrate(0.0, kDt);
        if (i > 3000) {
            low = std::min(low, engine.rpm());
            high = std::max(high, engine.rpm());
        }
    }
    CHECK(engine.running());
    CHECK_RANGE(low, 800.0, 900.0);
    CHECK_RANGE(high, 800.0, 900.0);
}

TEST(engineTorqueAndPowerCurve) {
    double peakTorque = 0.0, peakPower = 0.0, peakPowerRpm = 0.0;
    for (double rpm = 1000.0; rpm <= 7500.0; rpm += 250.0) {
        const double torque = dynoTorque(rpm);
        peakTorque = std::max(peakTorque, torque);
        if (torque * rpm * kRpmToRad > peakPower) {
            peakPower = torque * rpm * kRpmToRad;
            peakPowerRpm = rpm;
        }
    }
    CHECK_RANGE(peakTorque, 270.0, 330.0);
    CHECK_RANGE(peakPower / 1000.0, 160.0, 210.0);
    CHECK_RANGE(peakPowerRpm, 6000.0, 7400.0);
}

TEST(engineManifoldPressureFollowsThrottle) {
    Engine engine(EngineParams{});
    const auto settle = [&](double pedal) {
        for (int i = 0; i < 1500; ++i) {
            engine.forceSpeed(2500.0 * kRpmToRad);
            engine.update(pedal, 0.0, kDt);
        }
        return engine.readout().manifoldPressure;
    };
    const double closed = settle(0.0);
    const double part = settle(0.15);
    const double wide = settle(1.0);
    CHECK(closed < 45e3);
    CHECK(part > closed && part < wide);
    CHECK(wide > 0.97 * EngineParams{}.ambientPressure);
}

TEST(engineRevLimiterHoldsUnloadedEngine) {
    Engine engine(EngineParams{});
    double peak = 0.0;
    for (int i = 0; i < 5000; ++i) {
        engine.update(1.0, 0.0, kDt);
        engine.integrate(0.0, kDt);
        peak = std::max(peak, engine.rpm());
    }
    CHECK(peak < EngineParams{}.limiterRpm + 200.0);
    CHECK(engine.rpm() > EngineParams{}.limiterRpm - 600.0);
}

TEST(engineOverrunCutsFuelAndBrakes) {
    Engine engine(EngineParams{});
    for (int i = 0; i < 500; ++i) {
        engine.forceSpeed(4000.0 * kRpmToRad);
        engine.update(0.0, 0.0, kDt);
    }
    CHECK(engine.readout().fuelCut);
    CHECK_NEAR(engine.readout().fuelFlow, 0.0, 1e-12);
    CHECK(engine.torque() < -20.0);  // engine braking
}

TEST(engineStallsUnderLoadAndRestarts) {
    Engine engine(EngineParams{});
    for (int i = 0; i < 2000 && engine.running(); ++i) {
        engine.update(0.0, 0.0, kDt);
        engine.integrate(150.0, kDt);
    }
    CHECK(!engine.running());
    engine.setStarter(true);
    for (int i = 0; i < 3000 && !engine.running(); ++i) {
        engine.update(0.0, 0.0, kDt);
        engine.integrate(0.0, kDt);
    }
    CHECK(engine.running());
}
