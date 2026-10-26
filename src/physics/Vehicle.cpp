#include "slide/Vehicle.h"

#include <algorithm>
#include <cmath>

namespace slide {
namespace {

constexpr double kRollingResistanceSmoothing = 0.3;  // [m/s]

const AxleParams& axleFor(const VehicleParams& p, int corner) {
    return isFront(corner) ? p.front : p.rear;
}

}  // namespace

Vehicle::Vehicle(const VehicleParams& params, const Terrain* terrain)
    : params_(params),
      tireModels_{TireModel(params.frontTire), TireModel(params.rearTire)},
      suspension_(params),
      engine_(params.engine),
      drivetrain_(params.drivetrain),
      aids_(params.aids),
      terrain_(terrain) {
    reset({}, 0.0);
}

void Vehicle::reset(Vec2 position, double heading, double speed) {
    const Drivetrain::Mode mode = drivetrain_.mode();
    const DriverAidParams aidSettings = aids_.params();
    suspension_ = Suspension(params_);
    engine_ = Engine(params_.engine);
    drivetrain_ = Drivetrain(params_.drivetrain);
    drivetrain_.setMode(mode);
    aids_ = DriverAids(aidSettings);

    for (int i = 0; i < kCornerCount; ++i) {
        Wheel& w = wheels_[static_cast<size_t>(i)];
        const AxleParams& axle = axleFor(params_, i);
        w = Wheel{};
        w.position = suspension_.corner(i).position;
        w.radius = tireModel(i).params().radius;
        w.inertia = axle.wheelInertia;
        w.maxBrakeTorque = axle.maxBrakeTorque;
        w.omega = speed / w.radius;
        w.tire.gripLimit = suspension_.corner(i).tireLoad * tireModel(i).params().mu;
    }

    position_ = position;
    heading_ = heading;
    velocity_ = Vec2{std::cos(heading), std::sin(heading)} * speed;
    yawRate_ = 0.0;
    localAcceleration_ = {};
    steerInput_ = steerAngle_ = 0.0;
    frontLateral_ = rearLateral_ = 0.0;
    frontDownforce_ = rearDownforce_ = dragForce_ = 0.0;
    time_ = 0.0;
    firstStep_ = true;
    input_ = {};

    if (speed > 0.5) {
        // Rolling start: pick the lowest gear that keeps the engine below
        // 6000 rpm and match the crank to the wheels.
        const double wheelOmega = speed / params_.rearTire.radius;
        int gear = DrivetrainParams::kGearCount;
        for (int g = 1; g <= DrivetrainParams::kGearCount; ++g) {
            if (drivetrain_.ratioFor(g) * wheelOmega * kRadToRpm < 6000.0) {
                gear = g;
                break;
            }
        }
        drivetrain_.engage(gear);
        engine_.forceSpeed(drivetrain_.ratioFor(gear) * wheelOmega);
    }
}

double Vehicle::drivenWheelOmega() const {
    return 0.5 * (wheels_[kRearLeft].omega + wheels_[kRearRight].omega);
}

double Vehicle::sideslip() const {
    const Vec2 v = localVelocity();
    return v.lengthSq() > 0.25 ? std::atan2(v.y, std::abs(v.x)) : 0.0;
}

double Vehicle::maxSteerAngle() const {
    // Speed-sensitive lock, like a variable-ratio rack.
    const SteeringParams& p = params_.steering;
    return p.maxAngle * std::max(p.minSpeedFactor, 1.0 / (1.0 + sq(speed() / p.speedReference)));
}

void Vehicle::updateSteering(double input, double dt) {
    const SteeringParams& p = params_.steering;
    steerInput_ = approach(steerInput_, std::clamp(input, -1.0, 1.0), p.inputRate * dt);
    steerAngle_ = steerInput_ * maxSteerAngle();

    // Ackermann: the inside wheel follows a tighter circle about the rear axle.
    const double wheelbase = params_.chassis.wheelbase;
    const double halfTrack = 0.5 * params_.front.track;
    const double angle = std::abs(steerAngle_);
    double inner = angle;
    double outer = angle;
    if (angle > 1e-6) {
        const double radius = wheelbase / std::tan(angle);
        inner = lerp(angle, std::atan(wheelbase / (radius - halfTrack)), p.ackermann);
        outer = lerp(angle, std::atan(wheelbase / (radius + halfTrack)), p.ackermann);
    }
    const bool left = steerAngle_ > 0.0;
    const double s = sign(steerAngle_);
    wheels_[kFrontLeft].steerAngle = s * (left ? inner : outer);
    wheels_[kFrontRight].steerAngle = s * (left ? outer : inner);
}

void Vehicle::step(const DriverInput& rawInput, double dt) {
    DriverInput in = rawInput;
    in.throttle = std::clamp(in.throttle, 0.0, 1.0);
    in.brake = std::clamp(in.brake, 0.0, 1.0);
    in.clutch = std::clamp(in.clutch, 0.0, 1.0);
    input_ = in;
    time_ += dt;
    updateSteering(in.steer, dt);

    const Rot2 rot(heading_);
    const Vec2 vBody = rot.applyInverse(velocity_);

    // 1. Contact patch kinematics and the surface under each tire.
    std::array<double, kCornerCount> roadHeight{};
    std::array<double, kCornerCount> roadRate{};
    for (int i = 0; i < kCornerCount; ++i) {
        Wheel& w = wheels_[static_cast<size_t>(i)];
        const Vec2 contact{vBody.x - yawRate_ * w.position.y, vBody.y + yawRate_ * w.position.x};
        w.groundVelocity = Rot2(w.steerAngle).applyInverse(contact);
        w.surface = terrain().sample(position_ + rot.apply(w.position));
        roadRate[static_cast<size_t>(i)] = firstStep_ ? 0.0 : (w.surface.height - w.roadHeight) / dt;
        roadHeight[static_cast<size_t>(i)] = w.roadHeight = w.surface.height;
    }
    firstStep_ = false;

    // 2. Normal loads from the suspension's current state.
    suspension_.computeLoads(roadHeight, roadRate, frontLateral_, rearLateral_);

    // 3. Tire forces from the relaxed contact state.
    for (int i = 0; i < kCornerCount; ++i) {
        Wheel& w = wheels_[static_cast<size_t>(i)];
        w.tire = tireModel(i).evaluate(w.contact, w.groundVelocity.x, w.groundVelocity.y,
                                     w.omega * w.radius, suspension_.corner(i).tireLoad,
                                     w.surface.grip);
    }

    // 4. Driver aids see wheel speeds and body motion like real sensors would.
    AidSensors sensors;
    for (int i = 0; i < kCornerCount; ++i) {
        const Wheel& w = wheels_[static_cast<size_t>(i)];
        sensors.wheelSpeed[static_cast<size_t>(i)] = w.omega * w.radius;
        sensors.groundSpeed[static_cast<size_t>(i)] = w.groundVelocity.x;
    }
    sensors.forwardSpeed = vBody.x;
    sensors.lateralSpeed = vBody.y;
    sensors.yawRate = yawRate_;
    sensors.steerAngle = steerAngle_;
    sensors.wheelbase = params_.chassis.wheelbase;
    sensors.brakePedal = in.brake;
    sensors.throttle = in.throttle;
    aids_.update(sensors, dt);

    // 5. Engine and drivetrain. Pulling the handbrake also dips the clutch so
    //    the engine survives locking the rear wheels.
    drivetrain_.setClutchPedal(std::max(in.clutch, in.handbrake ? 1.0 : 0.0));
    drivetrain_.updateControl(engine_, in.throttle, in.brake, drivenWheelOmega(), dt);
    const double pedal = std::max(in.throttle, drivetrain_.shiftThrottleBlip());
    const double cut = std::max(aids_.torqueCut(), drivetrain_.shiftTorqueCut());
    engine_.update(pedal, cut, dt);

    std::array<double, kCornerCount> external{};
    for (int i = 0; i < kCornerCount; ++i) {
        const Wheel& w = wheels_[static_cast<size_t>(i)];
        const double load = suspension_.corner(i).tireLoad;
        const double rolling = w.surface.rollingResistance * load * w.radius *
                               std::tanh(w.omega * w.radius / kRollingResistanceSmoothing);
        external[static_cast<size_t>(i)] = -w.tire.Fx * w.radius - rolling;
    }
    const Drivetrain::AxleTorque axle =
        drivetrain_.solve(engine_, wheels_[kRearLeft].omega, wheels_[kRearRight].omega,
                          external[kRearLeft], external[kRearRight],
                          wheels_[kRearLeft].inertia, dt);
    engine_.integrate(drivetrain_.clutchTorque(), dt);
    wheels_[kRearLeft].driveTorque = axle.left;
    wheels_[kRearRight].driveTorque = axle.right;

    // 6. Wheel spin with Coulomb brakes: a brake can stop a wheel but never
    //    reverse it, so it is applied as a clamp after the free update.
    const double hydraulic = 1.0 - std::exp(-dt / params_.brakes.hydraulicTimeConstant);
    for (int i = 0; i < kCornerCount; ++i) {
        Wheel& w = wheels_[static_cast<size_t>(i)];
        double command = in.brake * w.maxBrakeTorque * aids_.brakeModulation(i) +
                         aids_.stabilityBrake(i);
        if (!isFront(i) && in.handbrake) command += params_.brakes.handbrakeTorque;
        w.brakeTorque += (command - w.brakeTorque) * hydraulic;

        w.omega += (external[static_cast<size_t>(i)] + w.driveTorque) / w.inertia * dt;
        const double brakeDelta = w.brakeTorque / w.inertia * dt;
        w.omega = std::abs(w.omega) <= brakeDelta ? 0.0 : w.omega - sign(w.omega) * brakeDelta;
        w.spinAngle = std::fmod(w.spinAngle + w.omega * dt, kTwoPi);

        // Relax with the updated spin rate (semi-implicit in the stiff
        // wheel/tire loop).
        tireModel(i).relax(w.contact, w.groundVelocity.x, w.groundVelocity.y,
                         w.omega * w.radius, dt);
    }

    // 7. Chassis: sum tire forces in the body frame, add aero, integrate.
    Vec2 force;
    double yawMoment = 0.0;
    std::array<Vec2, kCornerCount> tireForces{};
    for (int i = 0; i < kCornerCount; ++i) {
        Wheel& w = wheels_[static_cast<size_t>(i)];
        w.force = Vec2{w.tire.Fx, w.tire.Fy}.rotated(w.steerAngle);
        tireForces[static_cast<size_t>(i)] = w.force;
        force += w.force;
        yawMoment += cross(w.position, w.force);
    }
    frontLateral_ = tireForces[kFrontLeft].y + tireForces[kFrontRight].y;
    rearLateral_ = tireForces[kRearLeft].y + tireForces[kRearRight].y;

    const ChassisParams& c = params_.chassis;
    const double airspeed = vBody.length();
    const Vec2 drag = vBody * (-0.5 * c.airDensity * c.dragArea * airspeed);
    dragForce_ = drag.length();
    force += drag;
    const double downforce = 0.5 * c.airDensity * c.downforceArea * airspeed * airspeed;
    frontDownforce_ = downforce * c.aeroFrontBalance;
    rearDownforce_ = downforce - frontDownforce_;

    localAcceleration_ = force / c.mass;
    velocity_ += rot.apply(localAcceleration_) * dt;
    yawRate_ += yawMoment / c.yawInertia * dt;
    position_ += velocity_ * dt;
    heading_ = wrapAngle(heading_ + yawRate_ * dt);

    suspension_.integrate(tireForces, frontDownforce_, rearDownforce_, dt);
}

}  // namespace slide
