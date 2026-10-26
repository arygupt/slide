#include "slide/Tire.h"

#include <algorithm>
#include <cmath>

namespace slide {
namespace {

double magicFormula(double x, double B, double C, double E) {
    const double bx = B * x;
    return std::sin(C * std::atan(bx - E * (bx - std::atan(bx))));
}

// The curve peaks where C * atan(phi) = pi/2; phi(x) is monotonic for E < 1,
// so a bisection on x finds the peak without sampling.
double findPeak(double B, double C, double E) {
    if (C <= 1.0) return 1.0;  // monotonic curve, no interior peak
    const double target = std::tan(kPi / (2.0 * C));
    double lo = 0.0;
    double hi = 10.0;
    for (int i = 0; i < 80; ++i) {
        const double mid = 0.5 * (lo + hi);
        const double bx = B * mid;
        const double phi = bx - E * (bx - std::atan(bx));
        (phi < target ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
}

}  // namespace

TireModel::TireModel(const TireParams& params)
    : p_(params),
      peakSlipRatio_(findPeak(params.Bx, params.Cx, params.Ex)),
      peakSlipAngle_(findPeak(params.By, params.Cy, params.Ey)) {}

double TireModel::longitudinalCurve(double slipRatio) const {
    return magicFormula(slipRatio, p_.Bx, p_.Cx, p_.Ex);
}

double TireModel::lateralCurve(double slipAngle) const {
    return magicFormula(slipAngle, p_.By, p_.Cy, p_.Ey);
}

double TireModel::frictionCoefficient(double normalLoad, double surfaceGrip) const {
    const double loadRatio = normalLoad / p_.nominalLoad - 1.0;
    const double scale = std::max(0.5, 1.0 - p_.loadSensitivity * loadRatio);
    return p_.mu * scale * surfaceGrip;
}

TireOutput TireModel::forces(double slipRatio, double slipAngle, double normalLoad,
                             double surfaceGrip) const {
    TireOutput out;
    out.slipRatio = slipRatio;
    out.slipAngle = slipAngle;
    if (normalLoad <= 0.0) return out;

    const double grip = frictionCoefficient(normalLoad, surfaceGrip) * normalLoad;
    out.gripLimit = grip;

    // Similarity method: scale both slips to "fractions of peak", evaluate each
    // pure-slip curve at the combined magnitude, then share it by direction.
    const double sx = slipRatio / peakSlipRatio_;
    const double sy = slipAngle / peakSlipAngle_;
    const double s = std::max(std::hypot(sx, sy), 1e-9);
    out.Fx = grip * (sx / s) * longitudinalCurve(s * peakSlipRatio_);
    out.Fy = grip * (sy / s) * lateralCurve(s * peakSlipAngle_);
    out.utilization = std::hypot(out.Fx, out.Fy) / grip;
    return out;
}

void TireModel::relax(TireContact& contact, double vx, double vy, double wheelSpeed,
                      double dt) const {
    // sigma * d(kappa)/dt + |vx| * kappa = omega*R - vx, written in deflection
    // form u = sigma * kappa. The decay term is integrated implicitly so it is
    // unconditionally stable at any speed.
    const double speed = std::abs(vx);
    const double sx = p_.relaxationLengthX;
    const double sy = p_.relaxationLengthY;
    double ux = (contact.deflectionX + (wheelSpeed - vx) * dt) / (1.0 + speed * dt / sx);
    double uy = (contact.deflectionY - vy * dt) / (1.0 + speed * dt / sy);

    const double maxUx = sx * p_.maxSlipRatio;
    const double maxUy = sy * std::tan(p_.maxSlipAngle);
    contact.deflectionX = std::clamp(ux, -maxUx, maxUx);
    contact.deflectionY = std::clamp(uy, -maxUy, maxUy);
}

TireOutput TireModel::evaluate(const TireContact& contact, double vx, double vy,
                               double wheelSpeed, double normalLoad,
                               double surfaceGrip) const {
    const double slipRatio = contact.deflectionX / p_.relaxationLengthX;
    const double slipAngle = std::atan(contact.deflectionY / p_.relaxationLengthY);
    TireOutput out = forces(slipRatio, slipAngle, normalLoad, surfaceGrip);

    const double slipVx = vx - wheelSpeed;
    out.slipSpeed = std::hypot(slipVx, vy);
    if (normalLoad <= 0.0) return out;

    const double lowSpeed = 1.0 - smoothstep(0.0, p_.lowSpeedThreshold, std::abs(vx));
    if (lowSpeed > 0.0) {
        out.Fx -= p_.lowSpeedDampingX * lowSpeed * slipVx;
        out.Fy -= p_.lowSpeedDampingY * lowSpeed * vy;
        const double magnitude = std::hypot(out.Fx, out.Fy);
        if (magnitude > out.gripLimit) {
            const double scale = out.gripLimit / magnitude;
            out.Fx *= scale;
            out.Fy *= scale;
        }
        out.utilization = std::hypot(out.Fx, out.Fy) / out.gripLimit;
    }
    return out;
}

}  // namespace slide
