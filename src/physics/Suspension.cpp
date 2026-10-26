#include "slide/Suspension.h"

#include <algorithm>

namespace slide {

Suspension::Suspension(const VehicleParams& params)
    : front_(params.front),
      rear_(params.rear),
      sprungMass_(params.sprungMass()),
      pitchInertia_(params.chassis.pitchInertia),
      rollInertia_(params.chassis.rollInertia),
      cgHeight_(params.chassis.cgHeight),
      cgToFront_(params.chassis.cgToFront()),
      cgToRear_(params.chassis.cgToRear()) {
    const double wheelbase = params.chassis.wheelbase;
    const double weight = sprungMass_ * kGravity;
    for (int i = 0; i < kCornerCount; ++i) {
        CornerState& c = corners_[static_cast<size_t>(i)];
        const double halfTrack = 0.5 * axle(i).track;
        c.position = {isFront(i) ? cgToFront_ : -cgToRear_, isLeft(i) ? halfTrack : -halfTrack};
        // Static moment balance about the CG splits weight by axle distance.
        const double axleShare = isFront(i) ? cgToRear_ / wheelbase : cgToFront_ / wheelbase;
        c.staticLoad = 0.5 * weight * axleShare;
        c.strutForce = c.staticLoad;
        c.tireLoad = c.staticLoad + axle(i).unsprungMass * kGravity;
    }
}

void Suspension::computeLoads(const std::array<double, kCornerCount>& roadHeight,
                              const std::array<double, kCornerCount>& roadRate,
                              double frontLateral, double rearLateral) {
    for (int i = 0; i < kCornerCount; ++i) {
        CornerState& c = corners_[static_cast<size_t>(i)];
        const AxleParams& a = axle(i);
        // Small-angle body displacement at the corner.
        const double z = body_.heave - c.position.x * body_.pitch + c.position.y * body_.roll;
        const double zRate =
            body_.heaveRate - c.position.x * body_.pitchRate + c.position.y * body_.rollRate;
        c.compression = roadHeight[static_cast<size_t>(i)] - z;
        c.compressionRate = roadRate[static_cast<size_t>(i)] - zRate;

        c.springForce = a.springRate * c.compression;
        if (c.compression > a.bumpTravel) {
            c.springForce += a.bumpStopRate * sq(c.compression - a.bumpTravel);
        }
        const double damping = c.compressionRate > 0.0 ? a.bumpDamping : a.reboundDamping;
        c.damperForce = damping * c.compressionRate;
    }

    // Anti-roll bars push the more compressed side up and the other side down.
    for (int left : {kFrontLeft, kRearLeft}) {
        CornerState& l = corners_[static_cast<size_t>(left)];
        CornerState& r = corners_[static_cast<size_t>(left + 1)];
        const double force = axle(left).antiRollRate * (l.compression - r.compression);
        l.antiRollForce = force;
        r.antiRollForce = -force;
    }

    for (int i = 0; i < kCornerCount; ++i) {
        CornerState& c = corners_[static_cast<size_t>(i)];
        const AxleParams& a = axle(i);
        // A strut cannot pull the body down: past full droop the wheel hangs.
        c.strutForce = std::max(0.0, c.staticLoad + c.springForce + c.damperForce + c.antiRollForce);

        // Lateral force reacting through the roll center loads the outside
        // tire directly, without compressing the springs.
        const double lateral = isFront(i) ? frontLateral : rearLateral;
        const double geometric = lateral * a.rollCenterHeight / a.track;
        const double transfer = isLeft(i) ? -geometric : geometric;
        c.tireLoad = std::max(0.0, c.strutForce + a.unsprungMass * kGravity + transfer);
    }
}

void Suspension::integrate(const std::array<Vec2, kCornerCount>& tireForces,
                           double frontDownforce, double rearDownforce, double dt) {
    double heaveForce = -frontDownforce - rearDownforce;
    double pitchMoment = cgToFront_ * frontDownforce - cgToRear_ * rearDownforce;
    double rollMoment = 0.0;
    double totalLongitudinal = 0.0;

    for (int i = 0; i < kCornerCount; ++i) {
        const CornerState& c = corners_[static_cast<size_t>(i)];
        heaveForce += c.strutForce - c.staticLoad;
        pitchMoment -= c.position.x * c.strutForce;
        rollMoment += c.position.y * c.strutForce;
        totalLongitudinal += tireForces[static_cast<size_t>(i)].x;
        // Side force acts at the roll center, below the CG.
        rollMoment += (cgHeight_ - axle(i).rollCenterHeight) * tireForces[static_cast<size_t>(i)].y;
    }
    // Ground-plane tire forces sit cgHeight below the CG: squat and dive.
    pitchMoment -= cgHeight_ * totalLongitudinal;
    // Gravity acting through the displaced CG (destabilizing).
    const double meanRollCenter = 0.5 * (front_.rollCenterHeight + rear_.rollCenterHeight);
    rollMoment += sprungMass_ * kGravity * (cgHeight_ - meanRollCenter) * body_.roll;

    // Semi-implicit Euler.
    body_.heaveRate += heaveForce / sprungMass_ * dt;
    body_.pitchRate += pitchMoment / pitchInertia_ * dt;
    body_.rollRate += rollMoment / rollInertia_ * dt;
    body_.heave += body_.heaveRate * dt;
    body_.pitch += body_.pitchRate * dt;
    body_.roll += body_.rollRate * dt;
}

}  // namespace slide
