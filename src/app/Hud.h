#pragma once

#include <array>
#include <string>
#include <vector>

#include "raylib.h"
#include "slide/Vehicle.h"

namespace app {

struct FrameStats {
    double fps = 0.0;
    double frameMs = 0.0;
    double physicsHz = 0.0;
    double stepMicros = 0.0;     // wall time per physics step
    double stepsPerFrame = 0.0;
};

struct HudState {
    bool autopilot = false;
    bool recording = false;
    bool help = false;
    int laps = 0;
    double currentLap = 0.0;
    double lastLap = 0.0;
    double bestLap = 0.0;
    std::string toast;
    double toastAge = 1e9;
};

// Instrument cluster and engineering overlays: frame/physics timing, lap
// timer, tachometer, pedals, driver aids, per-tire friction circles with load
// and suspension travel, a G-G diagram, engine air path readouts and strip
// charts.
class Hud {
public:
    Hud();

    // Records history for the charts at a fixed 60 Hz, independent of FPS.
    void sample(const slide::Vehicle& vehicle, double frameDt);
    void clearHistory();
    void draw(const slide::Vehicle& vehicle, const FrameStats& stats, const HudState& state) const;

private:
    struct Snapshot {
        float speed, rpm, throttle, brake, lateralG, longitudinalG;
    };

    void drawPerformance(const FrameStats& stats, float x, float y) const;
    void drawLap(const HudState& state, float cx, float y) const;
    void drawTires(const slide::Vehicle& v, float right, float y) const;
    void drawDash(const slide::Vehicle& v, float cx, float bottom) const;
    void drawGg(float x, float bottom) const;
    void drawEngine(const slide::Vehicle& v, float right, float bottom) const;
    void drawHelp() const;

    float scale() const;
    const Snapshot& history(size_t ago) const;

    std::vector<Snapshot> history_;
    size_t head_ = 0;
    size_t count_ = 0;
    double sampleClock_ = 0.0;
};

}  // namespace app
