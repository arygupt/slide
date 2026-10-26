#pragma once

#include <vector>

#include "slide/Math.h"
#include "slide/Terrain.h"

namespace slide {

// Closed circuit built from a Catmull-Rom spline, resampled at even spacing.
// Doubles as the terrain: asphalt on the racing surface, rumble-strip curbs on
// corner exits and bumpy, low-grip grass everywhere else. A uniform grid of
// segment lists keeps per-wheel lookups to a handful of distance tests.
class Track final : public Terrain {
public:
    struct Sample {
        Vec2 point;
        Vec2 tangent;          // unit
        double distance = 0;   // arc length from the start line [m]
        double curvature = 0;  // signed, + = turning left [1/m]
    };

    struct Projection {
        double distance = 0.0;  // arc length of the closest point [m]
        double lateral = 0.0;   // signed offset from the centerline, + = left [m]
        bool nearTrack = false;
    };

    explicit Track(const std::vector<Vec2>& controlPoints, double halfWidth = 6.0,
                   double spacing = 2.0);

    // The default circuit (~1.9 km) used by the app and the test stand.
    static Track circuit();

    SurfaceSample sample(Vec2 worldPosition) const override;
    Projection project(Vec2 worldPosition) const;

    Vec2 pointAt(double distance) const;
    Vec2 tangentAt(double distance) const;
    double curvatureAt(double distance) const;
    double wrap(double distance) const;

    double length() const { return length_; }
    double halfWidth() const { return halfWidth_; }
    double curbWidth() const { return curbWidth_; }
    bool hasCurb(double distance) const;
    const std::vector<Sample>& samples() const { return samples_; }

private:
    void buildGrid();
    int cellIndex(int cx, int cy) const { return cy * gridWidth_ + cx; }

    std::vector<Sample> samples_;
    double length_ = 0.0;
    double halfWidth_;
    double curbWidth_ = 1.2;
    double spacing_;

    // Spatial grid over the circuit: cell -> range in gridSegments_.
    double cellSize_ = 10.0;
    Vec2 gridOrigin_;
    int gridWidth_ = 0;
    int gridHeight_ = 0;
    std::vector<int> cellStart_;
    std::vector<int> gridSegments_;
};

}  // namespace slide
