#include "Effects.h"

#include <algorithm>
#include <cmath>

#include "Draw.h"

namespace app {
namespace {

constexpr size_t kMaxMarks = 8000;
constexpr size_t kMaxSmoke = 700;
constexpr double kMarkSpacing = 0.25;     // [m] between mark segments
constexpr double kMarkSlipSpeed = 2.0;    // [m/s] contact sliding to leave rubber
constexpr double kSmokeSlipSpeed = 4.0;
constexpr float kMarkWidth = 0.22f;       // [m]

}  // namespace

Effects::Effects() : marks_(kMaxMarks), smoke_(kMaxSmoke) {}

void Effects::clear() {
    markHead_ = markCount_ = 0;
    marking_.fill(false);
    for (Particle& p : smoke_) p.life = 0.0;
}

void Effects::update(const slide::Vehicle& v, double dt) {
    using namespace slide;
    for (int i = 0; i < kCornerCount; ++i) {
        const Wheel& w = v.wheel(i);
        const Vec2 contact = v.worldPoint(w.position);
        const double load = w.tire.gripLimit > 0.0 ? v.suspension().corner(i).tireLoad : 0.0;
        const double slip = w.tire.slipSpeed;
        const bool grass = w.surface.kind == SurfaceKind::Grass;
        const bool sliding = load > 800.0 && (slip > kMarkSlipSpeed || (grass && v.speed() > 3.0));

        size_t idx = static_cast<size_t>(i);
        if (!sliding) {
            marking_[idx] = false;
        } else if (!marking_[idx]) {
            marking_[idx] = true;
            lastContact_[idx] = contact;
        } else if ((contact - lastContact_[idx]).length() > kMarkSpacing) {
            const float intensity =
                grass ? 0.35f : static_cast<float>(std::clamp((slip - kMarkSlipSpeed) / 8.0 + 0.25, 0.0, 0.85));
            const Color base = grass ? Color{70, 52, 30, 255} : Color{14, 14, 16, 255};
            marks_[markHead_] = {rl(lastContact_[idx]), rl(contact), withAlpha(base, intensity)};
            markHead_ = (markHead_ + 1) % kMaxMarks;
            markCount_ = std::min(markCount_ + 1, kMaxMarks);
            lastContact_[idx] = contact;
        }

        if (!grass && load > 800.0 && slip > kSmokeSlipSpeed) {
            smokeBudget_ += (slip - kSmokeSlipSpeed) * 6.0 * dt;
            while (smokeBudget_ >= 1.0) {
                smokeBudget_ -= 1.0;
                emitSmoke(contact, v.velocity() * 0.3, std::min(1.0, (slip - kSmokeSlipSpeed) / 10.0));
            }
        }
    }

    for (Particle& p : smoke_) {
        if (p.age >= p.life) continue;
        p.age += dt;
        p.position += p.velocity * dt;
        p.velocity *= std::exp(-1.5 * dt);
        p.size += 1.6 * dt;
    }
}

void Effects::emitSmoke(slide::Vec2 at, slide::Vec2 velocity, double intensity) {
    // Small LCG keeps the effect deterministic and cheap.
    const auto rnd = [this] {
        seed_ = seed_ * 1664525u + 1013904223u;
        return static_cast<double>(seed_ >> 8) / static_cast<double>(1u << 24) - 0.5;
    };
    Particle& p = smoke_[smokeHead_];
    smokeHead_ = (smokeHead_ + 1) % kMaxSmoke;
    p.position = at + slide::Vec2{rnd(), rnd()} * 0.4;
    p.velocity = velocity + slide::Vec2{rnd(), rnd()} * 1.5;
    p.age = 0.0;
    p.life = 1.2 + 1.5 * intensity + 0.4 * rnd();
    p.size = 0.4 + 0.3 * intensity;
}

void Effects::drawMarks() const {
    for (size_t k = 0; k < markCount_; ++k) {
        const Mark& m = marks_[(markHead_ + kMaxMarks - markCount_ + k) % kMaxMarks];
        DrawLineEx(m.a, m.b, kMarkWidth, m.color);
    }
}

void Effects::drawSmoke() const {
    for (const Particle& p : smoke_) {
        if (p.age >= p.life) continue;
        const float t = static_cast<float>(p.age / p.life);
        const float alpha = 0.28f * (1.0f - t) * std::min(1.0f, t * 8.0f);
        DrawCircleV(rl(p.position), static_cast<float>(p.size), withAlpha({215, 218, 222, 255}, alpha));
    }
}

}  // namespace app
