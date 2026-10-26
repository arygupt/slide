#pragma once

#include <vector>

#include "raylib.h"
#include "slide/Track.h"
#include "slide/Vehicle.h"

namespace app {

// Interpolated chassis pose between two physics steps.
struct CarPose {
    slide::Vec2 position;
    double heading = 0.0;

    slide::Vec2 toWorld(slide::Vec2 body) const { return position + slide::Rot2(heading).apply(body); }
};

class WorldRenderer {
public:
    explicit WorldRenderer(const slide::Track& track);

    void drawGround(const Camera2D& camera) const;
    void drawTrack() const;
    void drawCar(const slide::Vehicle& vehicle, const CarPose& pose, bool xray) const;
    void drawVectors(const slide::Vehicle& vehicle, const CarPose& pose) const;

private:
    struct Quad {
        Vector2 p[4];  // counter-clockwise in world space
        Color color;
    };

    void addQuad(slide::Vec2 a, slide::Vec2 b, slide::Vec2 c, slide::Vec2 d, Color color,
                 std::vector<Quad>& out) const;
    static void drawQuads(const std::vector<Quad>& quads);

    std::vector<Vector2> runoffStrip_;
    std::vector<Vector2> asphaltStrip_;
    std::vector<Vector2> leftEdge_;
    std::vector<Vector2> rightEdge_;
    std::vector<Quad> curbs_;
    std::vector<Quad> details_;  // asphalt patches, start line, distance boards
};

}  // namespace app
