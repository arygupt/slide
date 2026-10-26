#include "Check.h"
#include "slide/Tire.h"
#include "slide/Track.h"

using namespace slide;

TEST(tireZeroSlipGivesZeroForce) {
    const TireModel tire{TireParams{}};
    const TireOutput out = tire.forces(0.0, 0.0, 4000.0, 1.0);
    CHECK_NEAR(out.Fx, 0.0, 1e-9);
    CHECK_NEAR(out.Fy, 0.0, 1e-9);
}

TEST(tirePeakSlipIsRealistic) {
    const TireModel tire{TireParams{}};
    CHECK_RANGE(tire.peakSlipRatio(), 0.08, 0.20);
    CHECK_RANGE(toDeg(tire.peakSlipAngle()), 5.0, 11.0);
    // Past the peak the tire slides: force drops, but not off a cliff.
    CHECK_RANGE(tire.longitudinalCurve(1.0), 0.65, 0.9);
    CHECK_RANGE(tire.lateralCurve(deg(45.0)), 0.7, 0.95);
    CHECK_NEAR(tire.longitudinalCurve(tire.peakSlipRatio()), 1.0, 1e-6);
}

TEST(tireForcesAreOddInSlip) {
    const TireModel tire{TireParams{}};
    for (double k : {0.02, 0.1, 0.4}) {
        for (double a : {deg(1.0), deg(6.0), deg(20.0)}) {
            const TireOutput p = tire.forces(k, a, 4500.0, 1.0);
            const TireOutput n = tire.forces(-k, -a, 4500.0, 1.0);
            CHECK_NEAR(p.Fx, -n.Fx, 1e-9);
            CHECK_NEAR(p.Fy, -n.Fy, 1e-9);
        }
    }
}

TEST(tireCombinedForceStaysInsideFrictionEllipse) {
    const TireModel tire{TireParams{}};
    const double load = 5000.0;
    const double grip = tire.frictionCoefficient(load, 1.0) * load;
    for (double k = -1.0; k <= 1.0; k += 0.05) {
        for (double a = -0.6; a <= 0.6; a += 0.02) {
            const TireOutput out = tire.forces(k, a, load, 1.0);
            CHECK(std::hypot(out.Fx, out.Fy) <= grip * (1.0 + 1e-9));
        }
    }
    // Adding slip angle under braking costs longitudinal force.
    const double pure = std::abs(tire.forces(-0.1, 0.0, load, 1.0).Fx);
    const double combined = std::abs(tire.forces(-0.1, deg(6.0), load, 1.0).Fx);
    CHECK(combined < pure);
}

TEST(tireLoadSensitivity) {
    const TireModel tire{TireParams{}};
    CHECK(tire.frictionCoefficient(8000.0, 1.0) < tire.frictionCoefficient(2000.0, 1.0));
    const double light = tire.forces(0.0, deg(4.0), 2000.0, 1.0).Fy;
    const double heavy = tire.forces(0.0, deg(4.0), 8000.0, 1.0).Fy;
    CHECK(heavy > light);              // more load, more force...
    CHECK(heavy < 4.0 * light);        // ...but less than proportionally
    CHECK(tire.forces(0.0, deg(4.0), 4000.0, 0.5).Fy < tire.forces(0.0, deg(4.0), 4000.0, 1.0).Fy);
}

TEST(tireRelaxationConvergesToSteadySlip) {
    const TireModel tire{TireParams{}};
    TireContact contact;
    const double vx = 20.0, vy = -1.0, wheelSpeed = 21.0;
    for (int i = 0; i < 2000; ++i) tire.relax(contact, vx, vy, wheelSpeed, 0.001);
    const TireOutput out = tire.evaluate(contact, vx, vy, wheelSpeed, 4000.0, 1.0);
    CHECK_NEAR(out.slipRatio, (wheelSpeed - vx) / vx, 1e-6);
    CHECK_NEAR(std::tan(out.slipAngle), -vy / vx, 1e-6);
}

TEST(tireRelaxationIsBoundedAtStandstill) {
    const TireModel tire{TireParams{}};
    TireContact contact;
    for (int i = 0; i < 10000; ++i) tire.relax(contact, 0.0, 0.0, 30.0, 0.001);  // burnout
    CHECK(std::isfinite(contact.deflectionX));
    CHECK(contact.deflectionX <= tire.params().relaxationLengthX * tire.params().maxSlipRatio + 1e-12);
    const TireOutput out = tire.evaluate(contact, 0.0, 0.0, 30.0, 4000.0, 1.0);
    CHECK(out.Fx > 0.0);
    CHECK(out.utilization <= 1.0 + 1e-9);
}

TEST(trackGeometryAndSurfaces) {
    const Track track = Track::circuit();
    CHECK_RANGE(track.length(), 1000.0, 3000.0);
    for (double s = 0.0; s < track.length(); s += 37.0) {
        const Track::Projection p = track.project(track.pointAt(s));
        CHECK(p.nearTrack);
        CHECK_NEAR(p.lateral, 0.0, 0.05);
        CHECK(track.sample(track.pointAt(s)).kind == SurfaceKind::Asphalt);
        // A point to the left of the centerline has positive lateral offset.
        const Vec2 left = track.pointAt(s) + track.tangentAt(s).perp() * 3.0;
        CHECK_NEAR(track.project(left).lateral, 3.0, 0.2);
    }
    CHECK(track.sample({5000.0, 5000.0}).kind == SurfaceKind::Grass);
    const Vec2 a = track.pointAt(10.0);
    const Vec2 b = track.pointAt(10.0 + track.length());
    CHECK_NEAR((a - b).length(), 0.0, 1e-9);
}
