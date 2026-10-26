#pragma once

#include "slide/Config.h"
#include "slide/Math.h"

namespace slide {

// Transient contact-patch state. The deflections follow first-order
// relaxation-length dynamics, which keeps slip well defined at standstill and
// stops the stiff wheel/tire loop from blowing up at low speed.
struct TireContact {
    double deflectionX = 0.0;  // longitudinal [m]
    double deflectionY = 0.0;  // lateral [m]
};

struct TireOutput {
    double Fx = 0.0;           // wheel frame, + forward [N]
    double Fy = 0.0;           // wheel frame, + left [N]
    double slipRatio = 0.0;    // relaxed kappa
    double slipAngle = 0.0;    // relaxed alpha [rad]
    double slipSpeed = 0.0;    // contact patch sliding speed [m/s]
    double gripLimit = 0.0;    // mu * Fz [N]
    double utilization = 0.0;  // |F| / gripLimit
};

// Pacejka Magic Formula tire with load-sensitive friction and similarity-based
// combined slip: slips are normalized by their peak values so the force vector
// traces a friction ellipse instead of being clipped to a circle.
class TireModel {
public:
    explicit TireModel(const TireParams& params);

    const TireParams& params() const { return p_; }

    // Unit-peak pure-slip curves.
    double longitudinalCurve(double slipRatio) const;
    double lateralCurve(double slipAngle) const;

    double peakSlipRatio() const { return peakSlipRatio_; }
    double peakSlipAngle() const { return peakSlipAngle_; }

    // Effective friction coefficient after load sensitivity and surface grip.
    double frictionCoefficient(double normalLoad, double surfaceGrip) const;

    // Steady-state forces for a given slip ratio and slip angle.
    TireOutput forces(double slipRatio, double slipAngle, double normalLoad,
                      double surfaceGrip) const;

    // Advances the contact deflections for one step. `vx`/`vy` are contact
    // patch velocities in the wheel frame, `wheelSpeed` is omega * radius.
    void relax(TireContact& contact, double vx, double vy, double wheelSpeed,
               double dt) const;

    // Forces from the current relaxed contact state, including low-speed
    // contact damping.
    TireOutput evaluate(const TireContact& contact, double vx, double vy,
                        double wheelSpeed, double normalLoad,
                        double surfaceGrip) const;

private:
    TireParams p_;
    double peakSlipRatio_;
    double peakSlipAngle_;
};

}  // namespace slide
