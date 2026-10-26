#pragma once

#include <array>
#include <vector>

#include "raylib.h"
#include "slide/Vehicle.h"

namespace app {

// Skid marks and tire smoke driven by contact-patch slip. Both live in fixed
// pools so a long drift session never allocates.
class Effects {
public:
    Effects();

    void update(const slide::Vehicle& vehicle, double dt);
    void clear();
    void drawMarks() const;  // under the car
    void drawSmoke() const;  // over the car

private:
    struct Mark {
        Vector2 a, b;
        Color color;
    };
    struct Particle {
        slide::Vec2 position;
        slide::Vec2 velocity;
        double age = 0.0;
        double life = 0.0;
        double size = 0.0;
    };

    void emitSmoke(slide::Vec2 at, slide::Vec2 velocity, double intensity);

    std::vector<Mark> marks_;
    size_t markHead_ = 0;
    size_t markCount_ = 0;
    std::array<slide::Vec2, slide::kCornerCount> lastContact_{};
    std::array<bool, slide::kCornerCount> marking_{};

    std::vector<Particle> smoke_;
    size_t smokeHead_ = 0;
    double smokeBudget_ = 0.0;
    unsigned seed_ = 12345u;
};

}  // namespace app
