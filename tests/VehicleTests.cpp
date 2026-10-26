#include <algorithm>
#include <cstring>

#include "Check.h"
#include "slide/Autopilot.h"
#include "slide/Vehicle.h"

using namespace slide;

namespace {

constexpr double kDt = 0.001;

double totalLoad(const Vehicle& v) {
    double sum = 0.0;
    for (const auto& c : v.suspension().corners()) sum += c.tireLoad;
    return sum;
}

double frontLoad(const Vehicle& v) {
    return v.suspension().corner(kFrontLeft).tireLoad + v.suspension().corner(kFrontRight).tireLoad;
}

// Kinetic energy of everything that moves in the planar model.
double kineticEnergy(const Vehicle& v) {
    const VehicleParams& p = v.params();
    double e = 0.5 * p.chassis.mass * v.velocity().lengthSq() +
               0.5 * p.chassis.yawInertia * v.yawRate() * v.yawRate();
    for (const Wheel& w : v.wheels()) e += 0.5 * w.inertia * w.omega * w.omega;
    return e;
}

double timeTo100(double hz) {
    Vehicle v;
    DriverInput in;
    in.throttle = 1.0;
    while (v.speed() < 100.0 / 3.6 && v.time() < 20.0) v.step(in, 1.0 / hz);
    return v.time();
}

double stoppingDistance(double hz, bool abs) {
    VehicleParams p;
    p.aids.absEnabled = abs;
    Vehicle v(p);
    v.reset({}, 0.0, 100.0 / 3.6);
    DriverInput in;
    in.brake = 1.0;
    while (v.speed() > 0.05 && v.time() < 10.0) v.step(in, 1.0 / hz);
    return v.position().length();
}

}  // namespace

TEST(vehicleRestsInStaticEquilibrium) {
    Vehicle v;
    const double weight = v.params().chassis.mass * kGravity;
    CHECK_NEAR(totalLoad(v), weight, 1e-6 * weight);
    CHECK_NEAR(frontLoad(v) / weight, v.params().chassis.frontWeightFraction, 0.02);
    for (int i = 0; i < 5000; ++i) v.step({}, kDt);
    CHECK(v.position().length() < 0.01);
    CHECK(std::abs(v.heading()) < 1e-4);
    CHECK_NEAR(totalLoad(v), weight, 0.005 * weight);
    CHECK(v.engine().running());
}

TEST(simulationIsDeterministic) {
    const auto run = [] {
        Vehicle v;
        for (int i = 0; i < 20000; ++i) {
            const double t = i * kDt;
            DriverInput in;
            in.throttle = 0.5 + 0.5 * std::sin(0.7 * t);
            in.steer = 0.6 * std::sin(0.9 * t);
            in.brake = (i / 3000) % 3 == 2 ? 0.6 : 0.0;
            v.step(in, kDt);
        }
        return v;
    };
    const Vehicle a = run();
    const Vehicle b = run();
    const auto same = [](double x, double y) { return std::memcmp(&x, &y, sizeof x) == 0; };
    CHECK(same(a.position().x, b.position().x));
    CHECK(same(a.position().y, b.position().y));
    CHECK(same(a.heading(), b.heading()));
    CHECK(same(a.engine().rpm(), b.engine().rpm()));
    for (int i = 0; i < kCornerCount; ++i) CHECK(same(a.wheel(i).omega, b.wheel(i).omega));
}

TEST(brakingTransfersLoadForward) {
    Vehicle v;
    v.reset({}, 0.0, 20.0);
    const double staticFront = frontLoad(v);
    DriverInput in;
    in.brake = 0.35;
    for (int i = 0; i < 1500; ++i) v.step(in, kDt);
    const ChassisParams& c = v.params().chassis;
    const double decel = -v.localAcceleration().x;
    CHECK(decel > 3.0);
    // Steady state: dFz = m a h / L.
    const double expected = c.mass * decel * c.cgHeight / c.wheelbase;
    CHECK_NEAR(frontLoad(v) - staticFront, expected, 0.15 * expected);
    CHECK(v.suspension().body().pitch > 0.0);  // nose dives
}

TEST(corneringLoadsOutsideTiresAndRollsBody) {
    Vehicle v;
    v.reset({}, 0.0, 15.0);
    DriverInput in;
    in.steer = 0.25;  // left turn
    in.throttle = 0.3;
    for (int i = 0; i < 4000; ++i) v.step(in, kDt);
    CHECK(v.yawRate() > 0.1);
    CHECK(v.localAcceleration().y > 2.0);
    const auto& s = v.suspension();
    CHECK(s.corner(kFrontRight).tireLoad > s.corner(kFrontLeft).tireLoad);
    CHECK(s.corner(kRearRight).tireLoad > s.corner(kRearLeft).tireLoad);
    CHECK(s.body().roll > 0.0);  // left side up: rolls toward the outside
}

TEST(coastingNeverGainsEnergy) {
    Vehicle v;
    v.reset({}, 0.0, 30.0);
    v.drivetrain().engage(0);
    double previous = kineticEnergy(v);
    for (int i = 1; i <= 10000; ++i) {
        DriverInput in;
        in.steer = 0.15 * std::sin(i * kDt);
        v.step(in, kDt);
        if (i % 100 == 0) {
            const double e = kineticEnergy(v);
            CHECK(e <= previous + 1.0);
            previous = e;
        }
    }
    CHECK(v.speed() < 30.0);
}

TEST(absBeatsLockedWheelsAndKeepsSteering) {
    const double withAbs = stoppingDistance(1000.0, true);
    const double locked = stoppingDistance(1000.0, false);
    CHECK(withAbs < 0.92 * locked);
    CHECK_RANGE(withAbs, 30.0, 42.0);

    Vehicle v;
    v.reset({}, 0.0, 100.0 / 3.6);
    DriverInput in;
    in.brake = 1.0;
    bool frontLocked = false;
    while (v.speed() > 5.0) {
        v.step(in, kDt);
        if (v.time() > 0.2 && v.wheel(kFrontLeft).omega <= 0.0) frontLocked = true;
    }
    CHECK(!frontLocked);
}

TEST(tractionControlLimitsWheelspin) {
    const auto maxRearSlip = [](bool tcs) {
        VehicleParams p;
        p.aids.tcsEnabled = tcs;
        Vehicle v(p);
        DriverInput in;
        in.throttle = 1.0;
        double worst = 0.0;
        for (int i = 0; i < 2500; ++i) {
            v.step(in, kDt);
            if (v.speed() > 5.0) worst = std::max(worst, v.wheel(kRearLeft).tire.slipRatio);
        }
        return worst;
    };
    const double with = maxRearSlip(true);
    const double without = maxRearSlip(false);
    CHECK(with < 0.3);
    CHECK(without > with);
}

TEST(resultsConvergeWithTimestep) {
    CHECK_NEAR(timeTo100(1000.0), timeTo100(2000.0), 0.03 * timeTo100(2000.0));
    CHECK_NEAR(stoppingDistance(1000.0, true), stoppingDistance(2000.0, true),
               0.03 * stoppingDistance(2000.0, true));
}

TEST(straightLineIsStable) {
    Vehicle v;
    v.reset({}, 0.0, 100.0 / 3.6);
    DriverInput in;
    in.throttle = 0.3;
    for (int i = 0; i < 5000; ++i) v.step(in, kDt);
    CHECK(std::abs(v.position().y) < 0.05);
    CHECK(std::abs(toDeg(v.heading())) < 0.1);
}

TEST(handbrakeLocksRearAndRotatesCar) {
    Vehicle v;
    v.reset({}, 0.0, 15.0);
    DriverInput in;
    in.steer = 0.5;
    in.handbrake = true;
    bool rearLocked = false;
    double maxSideslip = 0.0;
    for (int i = 0; i < 1200; ++i) {
        v.step(in, kDt);
        if (v.wheel(kRearLeft).omega == 0.0) rearLocked = true;
        maxSideslip = std::max(maxSideslip, std::abs(v.sideslip()));
    }
    CHECK(rearLocked);
    CHECK(maxSideslip > deg(10.0));
    CHECK(v.engine().running());
}

TEST(autopilotLapsCircuitOnTrack) {
    const Track track = Track::circuit();
    Vehicle v({}, &track);
    const double start = track.length() - 20.0;
    v.reset(track.pointAt(start), std::atan2(track.tangentAt(start).y, track.tangentAt(start).x));
    Autopilot pilot(track);
    LapTimer timer(track);
    double worstOffset = 0.0;
    while (timer.laps() < 1 && v.time() < 120.0) {
        v.step(pilot.drive(v), kDt);
        timer.update(v.position(), v.time());
        worstOffset = std::max(worstOffset, std::abs(track.project(v.position()).lateral));
    }
    CHECK(timer.laps() == 1);
    CHECK_RANGE(timer.lastLap(), 50.0, 70.0);
    CHECK(worstOffset < track.halfWidth());
}

TEST(reverseGearDrivesBackward) {
    Vehicle v;
    for (int i = 0; i < 500; ++i) v.step({}, kDt);
    CHECK(v.drivetrain().requestGear(-1, v.drivenWheelOmega()));
    DriverInput in;
    in.throttle = 0.4;
    for (int i = 0; i < 4000; ++i) v.step(in, kDt);
    CHECK(v.drivetrain().gear() == -1);
    CHECK(v.localVelocity().x < -2.0);
    CHECK(v.engine().running());
}

TEST(randomInputsStayFiniteAndBounded) {
    // Ten minutes of abusive, aid-free driving over curbs and grass.
    const Track track = Track::circuit();
    VehicleParams p;
    p.aids.absEnabled = p.aids.tcsEnabled = p.aids.escEnabled = false;
    Vehicle v(p, &track);
    unsigned seed = 2024u;
    const auto rnd = [&seed] {
        seed = seed * 1664525u + 1013904223u;
        return static_cast<double>(seed >> 8) / static_cast<double>(1u << 24);
    };
    DriverInput in;
    bool finite = true;
    double maxSpeed = 0.0;
    for (int i = 0; i < 600000; ++i) {
        if (i % 400 == 0) {
            in.throttle = rnd() < 0.6 ? rnd() : 0.0;
            in.brake = rnd() < 0.25 ? rnd() : 0.0;
            in.steer = 2.0 * rnd() - 1.0;
            in.handbrake = rnd() < 0.1;
            in.clutch = rnd() < 0.05 ? 1.0 : 0.0;
            if (rnd() < 0.05) v.drivetrain().requestShift(rnd() < 0.5 ? 1 : -1, v.drivenWheelOmega());
        }
        v.step(in, kDt);
        const auto& body = v.suspension().body();
        finite = finite && std::isfinite(v.position().x) && std::isfinite(v.yawRate()) &&
                 std::isfinite(v.engine().rpm()) && std::isfinite(body.roll) && std::isfinite(body.pitch);
        maxSpeed = std::max(maxSpeed, v.speed());
        if (!finite) break;
    }
    CHECK(finite);
    CHECK(maxSpeed < 90.0);
    CHECK(std::abs(toDeg(v.suspension().body().roll)) < 15.0);
}
