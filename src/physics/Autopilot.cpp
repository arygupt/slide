#include "slide/Autopilot.h"

#include <algorithm>
#include <cmath>

namespace slide {

Autopilot::Autopilot(const Track& track, Settings settings) : track_(&track), settings_(settings) {
    const auto& samples = track.samples();
    const size_t n = samples.size();
    const double ds = track.length() / static_cast<double>(n);
    profile_.resize(n);
    for (size_t i = 0; i < n; ++i) {
        const double k = std::max(std::abs(samples[i].curvature), 1e-4);
        profile_[i] = std::min(settings.maxSpeed, std::sqrt(settings.lateralGrip * kGravity / k));
    }
    // Two laps of each pass so the limits carry across the start line.
    for (int lap = 0; lap < 2; ++lap) {
        for (size_t k = 0; k < n; ++k) {
            const size_t i = n - 1 - k;
            const double next = profile_[(i + 1) % n];
            profile_[i] = std::min(profile_[i], std::sqrt(next * next + 2.0 * settings.brakeDecel * ds));
        }
        for (size_t i = 0; i < n; ++i) {
            const double prev = profile_[(i + n - 1) % n];
            profile_[i] = std::min(profile_[i], std::sqrt(prev * prev + 2.0 * settings.driveAccel * ds));
        }
    }
}

double Autopilot::plannedSpeed(double distance) const {
    const double s = track_->wrap(distance) / track_->length() * static_cast<double>(profile_.size());
    const size_t i = static_cast<size_t>(s) % profile_.size();
    return lerp(profile_[i], profile_[(i + 1) % profile_.size()], s - std::floor(s));
}

DriverInput Autopilot::drive(const Vehicle& vehicle) {
    const Track::Projection here = track_->project(vehicle.position());
    if (here.nearTrack) progress_ = here.distance;

    const double speed = vehicle.speed();
    const double forward = vehicle.localVelocity().x;
    const double wheelbase = vehicle.params().chassis.wheelbase;

    // Pure pursuit from the rear axle toward a point on the centerline.
    const Vec2 rearAxle = vehicle.worldPoint({-vehicle.params().chassis.cgToRear(), 0.0});
    const double lookahead = std::clamp(5.0 + 0.35 * speed, 7.0, 28.0);
    const Vec2 target = track_->pointAt(progress_ + lookahead);
    const Vec2 local = Rot2(vehicle.heading()).applyInverse(target - rearAxle);
    const double alpha = std::atan2(local.y, local.x);
    const double delta = std::atan(2.0 * wheelbase * std::sin(alpha) / std::max(local.length(), 1.0));

    DriverInput input;
    input.steer = std::clamp(delta / vehicle.maxSteerAngle(), -1.0, 1.0);

    // Track the planned speed a little ahead to cover brake latency.
    targetSpeed_ = plannedSpeed(progress_ + 2.0 + 0.3 * speed);
    const double error = targetSpeed_ - forward;
    if (error > 0.0) {
        input.throttle = std::clamp(0.25 + 0.35 * error, 0.0, 1.0);
    } else if (error < -0.8) {
        input.brake = std::clamp(-0.12 * error, 0.0, 1.0);
    } else {
        input.throttle = 0.15;
    }
    return input;
}

bool LapTimer::update(Vec2 position, double time) {
    const Track::Projection p = track_->project(position);
    if (!p.nearTrack) return false;
    const double length = track_->length();
    const double d = p.distance;
    if (lastDistance_ < 0.0) {
        lastDistance_ = d;
        return false;
    }
    const bool crossed = lastDistance_ > length - 30.0 && d < 30.0;
    if (d > 0.4 * length && d < 0.6 * length) armed_ = true;
    lastDistance_ = d;
    if (!crossed) return false;

    bool completed = false;
    if (started_ && armed_) {
        lastLap_ = time - lapStart_;
        bestLap_ = laps_ == 0 ? lastLap_ : std::min(bestLap_, lastLap_);
        ++laps_;
        completed = true;
    }
    started_ = true;
    armed_ = false;
    lapStart_ = time;
    return completed;
}

}  // namespace slide
