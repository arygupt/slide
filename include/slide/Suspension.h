#pragma once

#include <array>

#include "slide/Config.h"
#include "slide/Math.h"

namespace slide {

enum Corner : int { kFrontLeft = 0, kFrontRight = 1, kRearLeft = 2, kRearRight = 3 };
inline constexpr int kCornerCount = 4;

constexpr bool isFront(int corner) { return corner < 2; }
constexpr bool isLeft(int corner) { return (corner & 1) == 0; }

// Sprung mass with heave, pitch and roll on four independent spring/damper
// corners plus anti-roll bars. Tire normal loads come from the suspension
// forces, so load transfer is dynamic: the body has to actually pitch and roll
// before the tires feel it, and dampers set how quickly that happens.
//
// Body frame is x forward, y left, z up. Positive pitch is nose down, positive
// roll is left side up.
class Suspension {
public:
    struct BodyState {
        double heave = 0.0, heaveRate = 0.0;  // [m], [m/s]
        double pitch = 0.0, pitchRate = 0.0;  // [rad], [rad/s]
        double roll = 0.0, rollRate = 0.0;
    };

    struct CornerState {
        Vec2 position;                 // contact patch in body frame, from CG [m]
        double staticLoad = 0.0;       // sprung share of weight at rest [N]
        double compression = 0.0;      // from static ride height, + = bump [m]
        double compressionRate = 0.0;  // [m/s]
        double springForce = 0.0;      // includes bump stop [N]
        double damperForce = 0.0;
        double antiRollForce = 0.0;
        double strutForce = 0.0;       // total force on the body [N]
        double tireLoad = 0.0;         // normal load at the contact patch [N]
    };

    explicit Suspension(const VehicleParams& params);

    // Computes strut forces and tire normal loads. `frontLateral` and
    // `rearLateral` are axle side forces (body frame) used for the geometric
    // load transfer through the roll centers.
    void computeLoads(const std::array<double, kCornerCount>& roadHeight,
                      const std::array<double, kCornerCount>& roadRate, double frontLateral,
                      double rearLateral);

    // Integrates the sprung body given tire forces in the body frame and
    // aerodynamic downforce at each axle.
    void integrate(const std::array<Vec2, kCornerCount>& tireForces, double frontDownforce,
                   double rearDownforce, double dt);

    const BodyState& body() const { return body_; }
    const CornerState& corner(int i) const { return corners_[static_cast<size_t>(i)]; }
    const std::array<CornerState, kCornerCount>& corners() const { return corners_; }

private:
    const AxleParams& axle(int corner) const { return isFront(corner) ? front_ : rear_; }

    AxleParams front_;
    AxleParams rear_;
    double sprungMass_;
    double pitchInertia_;
    double rollInertia_;
    double cgHeight_;
    double cgToFront_;
    double cgToRear_;
    BodyState body_;
    std::array<CornerState, kCornerCount> corners_;
};

}  // namespace slide
