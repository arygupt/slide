#pragma once

#include <vector>

#include "slide/Track.h"
#include "slide/Vehicle.h"

namespace slide {

// Driver model that laps a Track: pure-pursuit steering on the centerline and
// a speed profile from lateral grip with forward/backward acceleration passes,
// the same construction used for quasi-steady-state lap time simulation.
class Autopilot {
public:
    struct Settings {
        double lateralGrip = 0.95;   // fraction of g used in corners
        double brakeDecel = 9.0;     // [m/s^2]
        double driveAccel = 5.0;     // [m/s^2], planning only
        double maxSpeed = 70.0;      // [m/s]
    };

    explicit Autopilot(const Track& track) : Autopilot(track, Settings{}) {}
    Autopilot(const Track& track, Settings settings);

    DriverInput drive(const Vehicle& vehicle);

    double targetSpeed() const { return targetSpeed_; }
    double progress() const { return progress_; }  // arc length of the car [m]
    double plannedSpeed(double distance) const;

private:
    const Track* track_;
    Settings settings_;
    std::vector<double> profile_;
    double targetSpeed_ = 0.0;
    double progress_ = 0.0;
};

// Lap timing from track progress, robust to the car reversing across the line.
class LapTimer {
public:
    explicit LapTimer(const Track& track) : track_(&track) {}

    // Returns true when a lap is completed on this update.
    bool update(Vec2 position, double time);

    int laps() const { return laps_; }
    double lastLap() const { return lastLap_; }
    double bestLap() const { return bestLap_; }
    double currentLap(double time) const { return started_ ? time - lapStart_ : 0.0; }

private:
    const Track* track_;
    double lastDistance_ = -1.0;
    double lapStart_ = 0.0;
    double lastLap_ = 0.0;
    double bestLap_ = 0.0;
    int laps_ = 0;
    bool started_ = false;
    bool armed_ = false;
};

}  // namespace slide
