#include "slide/Tire.h"

#include <algorithm>
#include <cmath>

namespace slide {
namespace {

// Unit-peak grip curve: a quarter sine up to the peak, then a quarter sine
// down to the sliding grip. Odd in x, so negative slip gives negative force.
double sineCurve(double x, double peak, double slide, double slideGrip) {
    const double s = std::abs(x) / peak;
    const double shape = s <= 1.0
        ? std::sin(0.5 * kPi * s)
        : 1.0 - (1.0 - slideGrip) * std::sin(0.5 * kPi * std::min(1.0, (s - 1.0) / (slide / peak - 1.0)));
    return std::copysign(shape, x);
}

}  // namespace

TireModel::TireModel(const TireParams& params)
    : p_(params) {}

double TireModel::longitudinalCurve(double slipRatio) const {
    return sineCurve(slipRatio, p_.peakSlipRatio, p_.slideSlipRatio, p_.slideGripX);
}

double TireModel::lateralCurve(double slipAngle) const {
    return sineCurve(slipAngle, p_.peakSlipAngle, p_.slideSlipAngle, p_.slideGripY);
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
    const double sx = slipRatio / p_.peakSlipRatio;
    const double sy = slipAngle / p_.peakSlipAngle;
    const double s = std::max(std::hypot(sx, sy), 1e-9);
    out.Fx = grip * (sx / s) * longitudinalCurve(s * p_.peakSlipRatio);
    out.Fy = grip * (sy / s) * lateralCurve(s * p_.peakSlipAngle);
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
