#include <algorithm>
#include <vector>

#include "Check.h"
#include "slide/Vehicle.h"

using namespace slide;

namespace {
constexpr double kDt = 0.001;
}

TEST(gearRatioSigns) {
    const Drivetrain d{DrivetrainParams{}};
    CHECK(d.ratioFor(0) == 0.0);
    CHECK(d.ratioFor(-1) < 0.0);
    for (int g = 1; g < DrivetrainParams::kGearCount; ++g) CHECK(d.ratioFor(g) > d.ratioFor(g + 1));
}

TEST(downshiftOverRevIsRefused) {
    Drivetrain d{DrivetrainParams{}};
    d.engage(4);
    const double wheelOmega = 7000.0 * kRpmToRad / d.ratioFor(4);  // 7000 rpm in 4th
    CHECK(!d.requestGear(2, wheelOmega));
    CHECK(d.requestGear(3, 3000.0 * kRpmToRad / d.ratioFor(4)));
}

TEST(reverseNeedsStandstill) {
    Drivetrain d{DrivetrainParams{}};
    d.engage(1);
    CHECK(!d.requestGear(-1, 20.0));
    CHECK(d.requestGear(-1, 0.0));
}

TEST(clutchLocksEngineToWheels) {
    Vehicle v;
    v.drivetrain().setMode(Drivetrain::Mode::Manual);  // no shifts mid-test
    v.reset({}, 0.0, 8.0);
    DriverInput in;
    in.throttle = 0.3;
    double worstSlip = 0.0;
    for (int i = 0; i < 3000; ++i) {
        v.step(in, kDt);
        worstSlip = std::max(worstSlip, std::abs(v.drivetrain().clutchSlip()));
    }
    CHECK(v.drivetrain().gear() == 1);
    CHECK(worstSlip < 1.0);  // rad/s
    const double expected = v.drivetrain().ratio() * v.drivenWheelOmega() * kRadToRpm;
    CHECK_NEAR(v.engine().rpm(), expected, 15.0);
}

TEST(automaticUpshiftsInOrderWithoutOverRevving) {
    Vehicle v;
    DriverInput in;
    in.throttle = 1.0;
    std::vector<int> gears{v.drivetrain().gear()};
    double peakRpm = 0.0;
    for (int i = 0; i < 25000; ++i) {
        v.step(in, kDt);
        peakRpm = std::max(peakRpm, v.engine().rpm());
        if (v.drivetrain().gear() != gears.back()) gears.push_back(v.drivetrain().gear());
    }
    CHECK(gears.size() >= 5);
    for (size_t i = 1; i < gears.size(); ++i) CHECK(gears[i] == gears[i - 1] + 1);
    CHECK(peakRpm < EngineParams{}.limiterRpm + 150.0);
}

TEST(brakingToAStopKeepsEngineAliveAndSelectsFirst) {
    Vehicle v;
    v.reset({}, 0.0, 120.0 / 3.6);
    DriverInput in;
    in.brake = 1.0;
    for (int i = 0; i < 8000; ++i) v.step(in, kDt);
    CHECK(v.speed() < 0.1);
    CHECK(v.engine().running());
    CHECK_RANGE(v.engine().rpm(), 700.0, 1000.0);
    in.brake = 0.0;
    for (int i = 0; i < 1500; ++i) v.step(in, kDt);
    CHECK(v.drivetrain().gear() == 1);
}

TEST(limitedSlipBiasesTorqueToSlowerWheel) {
    Drivetrain d{DrivetrainParams{}};
    Engine engine{EngineParams{}};
    d.engage(1);
    engine.forceSpeed(d.ratioFor(1) * 10.0);
    engine.update(1.0, 0.0, kDt);
    // Left wheel spinning faster: more torque should go to the right.
    const Drivetrain::AxleTorque t = d.solve(engine, 12.0, 8.0, -500.0, -500.0, 1.5, kDt);
    CHECK(t.right > t.left);
    CHECK(d.lsdTorque() > 0.0);
}
