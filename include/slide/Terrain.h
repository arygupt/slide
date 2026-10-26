#pragma once

#include <cstdint>

#include "slide/Math.h"

namespace slide {

enum class SurfaceKind : std::uint8_t { Asphalt, Curb, Grass };

struct SurfaceSample {
    double height = 0.0;             // road height under the contact patch [m]
    double grip = 1.0;               // friction scale relative to dry asphalt
    double rollingResistance = 0.012;
    SurfaceKind kind = SurfaceKind::Asphalt;
};

// What the vehicle drives on. Queried once per wheel per physics step, so
// implementations should be cheap.
class Terrain {
public:
    virtual ~Terrain() = default;
    virtual SurfaceSample sample(Vec2 worldPosition) const = 0;
};

class FlatTerrain final : public Terrain {
public:
    explicit FlatTerrain(SurfaceSample surface = {}) : surface_(surface) {}
    SurfaceSample sample(Vec2) const override { return surface_; }

private:
    SurfaceSample surface_;
};

}  // namespace slide
