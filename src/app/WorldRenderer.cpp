#include "WorldRenderer.h"

#include <algorithm>
#include <cmath>

#include "Draw.h"

namespace app {
namespace {

using slide::Vec2;

constexpr Color kGrassA{37, 72, 43, 255};
constexpr Color kGrassB{41, 79, 47, 255};
constexpr Color kRunoff{58, 66, 52, 255};
constexpr Color kAsphalt{54, 57, 63, 255};
constexpr Color kAsphaltPatch{49, 52, 58, 255};
constexpr Color kLine{226, 228, 230, 235};
constexpr Color kCurbRed{196, 42, 42, 255};
constexpr Color kCurbWhite{232, 232, 232, 255};
constexpr Color kBody{16, 185, 129, 255};
constexpr Color kBodyDark{6, 120, 87, 255};
constexpr Color kGlass{18, 24, 32, 255};

// Car silhouette in the body frame (x forward, y left), counter-clockwise.
constexpr Vec2 kOutline[] = {
    {2.175, 0.0},   {2.13, 0.55},   {1.95, 0.88},   {1.62, 0.91},  {-1.78, 0.91},
    {-2.07, 0.81},  {-2.175, 0.55}, {-2.175, -0.55}, {-2.07, -0.81}, {-1.78, -0.91},
    {1.62, -0.91},  {1.95, -0.88},  {2.13, -0.55},
};

void fan(const CarPose& pose, Vec2 offset, const Vec2* points, int count, double scale, Color color) {
    Vector2 v[20];
    v[0] = rl(pose.toWorld(offset));
    for (int i = 0; i < count; ++i) v[i + 1] = rl(pose.toWorld(offset + points[i] * scale));
    v[count + 1] = v[1];
    DrawTriangleFan(v, count + 2, color);
}

void quadBody(const CarPose& pose, Vec2 offset, Vec2 a, Vec2 b, Vec2 c, Vec2 d, Color color) {
    const Vector2 pa = rl(pose.toWorld(offset + a));
    const Vector2 pb = rl(pose.toWorld(offset + b));
    const Vector2 pc = rl(pose.toWorld(offset + c));
    const Vector2 pd = rl(pose.toWorld(offset + d));
    DrawTriangle(pa, pb, pc, color);
    DrawTriangle(pa, pc, pd, color);
}

void arrow(Vec2 from, Vec2 vector, float thickness, Color color) {
    const double length = vector.length();
    if (length < 0.05) return;
    const Vec2 to = from + vector;
    const Vec2 dir = vector / length;
    const double head = std::min(0.45, 0.35 * length);
    DrawLineEx(rl(from), rl(to - dir * head * 0.6), thickness, color);
    const Vec2 side = dir.perp() * head * 0.5;
    DrawTriangle(rl(to), rl(to - dir * head + side), rl(to - dir * head - side), color);
}

}  // namespace

WorldRenderer::WorldRenderer(const slide::Track& track) {
    const auto& samples = track.samples();
    const double hw = track.halfWidth();
    const double runoff = hw + track.curbWidth() + 2.5;
    for (size_t k = 0; k <= samples.size(); ++k) {
        const auto& s = samples[k % samples.size()];
        const Vec2 n = s.tangent.perp();
        runoffStrip_.push_back(rl(s.point + n * runoff));
        runoffStrip_.push_back(rl(s.point - n * runoff));
        asphaltStrip_.push_back(rl(s.point + n * hw));
        asphaltStrip_.push_back(rl(s.point - n * hw));
        leftEdge_.push_back(rl(s.point + n * (hw - 0.35)));
        rightEdge_.push_back(rl(s.point - n * (hw - 0.35)));
    }

    // Curbs: alternating red/white blocks on both edges through corners.
    for (double d = 0.0; d < track.length(); d += 1.0) {
        if (!track.hasCurb(d)) continue;
        const Vec2 p0 = track.pointAt(d), p1 = track.pointAt(d + 1.0);
        const Vec2 n0 = track.tangentAt(d).perp(), n1 = track.tangentAt(d + 1.0).perp();
        const Color c = static_cast<int>(d) % 2 == 0 ? kCurbRed : kCurbWhite;
        const double inner = hw, outer = hw + track.curbWidth();
        addQuad(p0 + n0 * inner, p0 + n0 * outer, p1 + n1 * outer, p1 + n1 * inner, c, curbs_);
        addQuad(p0 - n0 * outer, p0 - n0 * inner, p1 - n1 * inner, p1 - n1 * outer, c, curbs_);
    }

    // Asphalt patches so the surface reads as moving at speed.
    unsigned seed = 7u;
    const auto rnd = [&seed] {
        seed = seed * 1664525u + 1013904223u;
        return static_cast<double>(seed >> 8) / static_cast<double>(1u << 24);
    };
    for (double d = 0.0; d < track.length(); d += 4.0) {
        const double along = d + rnd() * 3.0;
        const double across = (rnd() * 2.0 - 1.0) * (hw - 1.5);
        const double len = 1.5 + rnd() * 3.0, wid = 0.6 + rnd() * 1.4;
        const Vec2 c = track.pointAt(along) + track.tangentAt(along).perp() * across;
        const Vec2 t = track.tangentAt(along) * (0.5 * len), n = track.tangentAt(along).perp() * (0.5 * wid);
        addQuad(c - t - n, c + t - n, c + t + n, c - t + n, kAsphaltPatch, details_);
    }

    // Checkered start/finish line.
    const Vec2 t0 = track.tangentAt(0.0), n0 = t0.perp();
    const double cell = 0.75;
    for (int row = 0; row < 2; ++row) {
        for (double y = -hw; y < hw - 1e-6; y += cell) {
            const bool white = ((static_cast<int>((y + hw) / cell) + row) % 2) == 0;
            const Vec2 base = track.pointAt(0.0) + t0 * (row * cell) + n0 * y;
            addQuad(base, base + t0 * cell, base + t0 * cell + n0 * cell, base + n0 * cell,
                    white ? kCurbWhite : Color{20, 20, 22, 255}, details_);
        }
    }

    // Distance boards every 100 m on both sides.
    for (double d = 100.0; d < track.length(); d += 100.0) {
        const Vec2 p = track.pointAt(d), t = track.tangentAt(d) * 0.4, n = track.tangentAt(d).perp();
        for (double side : {1.0, -1.0}) {
            const Vec2 c = p + n * side * (hw + track.curbWidth() + 3.5);
            const Vec2 w = n * 0.6;
            addQuad(c - t - w, c + t - w, c + t + w, c - t + w, kCurbWhite, details_);
        }
    }
}

void WorldRenderer::addQuad(Vec2 a, Vec2 b, Vec2 c, Vec2 d, Color color, std::vector<Quad>& out) const {
    // Enforce counter-clockwise winding (raylib culls back faces).
    if (cross(b - a, c - a) < 0.0) std::swap(b, d);
    out.push_back({{rl(a), rl(b), rl(c), rl(d)}, color});
}

void WorldRenderer::drawQuads(const std::vector<Quad>& quads) {
    for (const Quad& q : quads) {
        DrawTriangle(q.p[0], q.p[1], q.p[2], q.color);
        DrawTriangle(q.p[0], q.p[2], q.p[3], q.color);
    }
}

void WorldRenderer::drawGround(const Camera2D& camera) const {
    ClearBackground(kGrassA);
    const float w = static_cast<float>(GetScreenWidth()), h = static_cast<float>(GetScreenHeight());
    Vector2 lo{1e9f, 1e9f}, hi{-1e9f, -1e9f};
    for (Vector2 corner : {Vector2{0, 0}, Vector2{w, 0}, Vector2{0, h}, Vector2{w, h}}) {
        const Vector2 p = GetScreenToWorld2D(corner, camera);
        lo = {std::min(lo.x, p.x), std::min(lo.y, p.y)};
        hi = {std::max(hi.x, p.x), std::max(hi.y, p.y)};
    }
    // Mowing stripes, then a faint 10 m survey grid.
    constexpr float kStripe = 12.0f;
    for (float x = std::floor(lo.x / kStripe) * kStripe; x < hi.x; x += kStripe) {
        if (static_cast<int>(std::floor(x / kStripe)) % 2 == 0) {
            DrawRectangleRec({x, lo.y, kStripe, hi.y - lo.y}, kGrassB);
        }
    }
    constexpr float kGrid = 10.0f;
    const Color grid = withAlpha(BLACK, 0.06f);
    for (float x = std::floor(lo.x / kGrid) * kGrid; x < hi.x; x += kGrid) {
        DrawLineEx({x, lo.y}, {x, hi.y}, 0.08f, grid);
    }
    for (float y = std::floor(lo.y / kGrid) * kGrid; y < hi.y; y += kGrid) {
        DrawLineEx({lo.x, y}, {hi.x, y}, 0.08f, grid);
    }
}

void WorldRenderer::drawTrack() const {
    DrawTriangleStrip(runoffStrip_.data(), static_cast<int>(runoffStrip_.size()), kRunoff);
    DrawTriangleStrip(asphaltStrip_.data(), static_cast<int>(asphaltStrip_.size()), kAsphalt);
    drawQuads(details_);
    drawQuads(curbs_);
    DrawSplineLinear(leftEdge_.data(), static_cast<int>(leftEdge_.size()), 0.18f, kLine);
    DrawSplineLinear(rightEdge_.data(), static_cast<int>(rightEdge_.size()), 0.18f, kLine);
}

void WorldRenderer::drawCar(const slide::Vehicle& v, const CarPose& pose, bool xray) const {
    using namespace slide;
    constexpr int kCount = static_cast<int>(std::size(kOutline));

    // Exaggerate body motion so pitch and roll read at a glance.
    const auto& body = v.suspension().body();
    const double h = v.params().chassis.cgHeight * 4.0;
    const Vec2 lean{h * body.pitch, -h * body.roll};

    CarPose shadowPose = pose;
    shadowPose.position += Vec2{0.28, -0.36};
    fan(shadowPose, {}, kOutline, kCount, 1.03, withAlpha(BLACK, 0.35f));

    const auto drawWheels = [&](bool overlay) {
        for (int i = 0; i < kCornerCount; ++i) {
            const Wheel& w = v.wheel(i);
            const TireParams& tp = v.tireModel(i).params();
            const Vec2 c = pose.toWorld(w.position);
            const float len = static_cast<float>(1.9 * tp.radius), wid = static_cast<float>(tp.width);
            const Rectangle r{static_cast<float>(c.x), static_cast<float>(-c.y), len, wid};
            const float angle = rlAngle(pose.heading + w.steerAngle);
            if (overlay) {
                DrawRectanglePro({r.x, r.y, len + 0.12f, wid + 0.12f}, {0.5f * (len + 0.12f), 0.5f * (wid + 0.12f)},
                                 angle, heat(w.tire.utilization));
            }
            DrawRectanglePro(r, {0.5f * len, 0.5f * wid}, angle, Color{24, 24, 26, 255});
            if (w.brakeTorque > 100.0) {
                const float glow = static_cast<float>(std::min(1.0, w.brakeTorque / 2000.0));
                DrawRectanglePro({r.x, r.y, len * 0.5f, wid * 0.4f}, {0.25f * len, 0.2f * wid}, angle,
                                 withAlpha(palette::kRed, 0.7f * glow));
            }
        }
    };
    if (!xray) drawWheels(false);

    fan(pose, lean, kOutline, kCount, 1.0, kBodyDark);
    fan(pose, lean, kOutline, kCount, 0.95, kBody);
    // Glass and roof.
    quadBody(pose, lean, {1.05, 0.70}, {0.35, 0.74}, {0.35, -0.74}, {1.05, -0.70}, kGlass);
    quadBody(pose, lean, {0.30, 0.70}, {-0.95, 0.70}, {-0.95, -0.70}, {0.30, -0.70}, kBodyDark);
    quadBody(pose, lean, {-1.0, 0.70}, {-1.55, 0.62}, {-1.55, -0.62}, {-1.0, -0.70}, kGlass);
    // Twin stripes.
    for (double y : {0.12, -0.24}) {
        quadBody(pose, lean, {2.12, y + 0.12}, {2.12, y}, {-2.15, y}, {-2.15, y + 0.12},
                 withAlpha(RAYWHITE, 0.85f));
    }
    // Lights.
    const Color head{255, 244, 200, 255};
    const bool braking = v.lastInput().brake > 0.05 || v.lastInput().handbrake;
    const Color tail = braking ? Color{255, 60, 50, 255} : Color{130, 20, 20, 255};
    for (double s : {1.0, -1.0}) {
        quadBody(pose, lean, {2.08, s * 0.80}, {1.98, s * 0.88}, {1.95, s * 0.62}, {2.12, s * 0.58}, head);
        quadBody(pose, lean, {-2.10, s * 0.82}, {-2.15, s * 0.60}, {-2.07, s * 0.60}, {-2.02, s * 0.82}, tail);
    }
    if (braking) {
        for (double s : {1.0, -1.0}) {
            DrawCircleV(rl(pose.toWorld(lean + Vec2{-2.3, s * 0.72})), 0.45f, withAlpha(palette::kRed, 0.25f));
        }
    }

    if (xray) drawWheels(true);
}

void WorldRenderer::drawVectors(const slide::Vehicle& v, const CarPose& pose) const {
    using namespace slide;
    const Rot2 rot(pose.heading);
    for (int i = 0; i < kCornerCount; ++i) {
        const Wheel& w = v.wheel(i);
        arrow(pose.toWorld(w.position), rot.apply(w.force) / 2500.0, 0.12f, palette::kBlue);
    }
    arrow(pose.position, v.velocity() * 0.12, 0.14f, palette::kAccent);
    arrow(pose.position, rot.apply(v.localAcceleration()) * 0.35, 0.14f, palette::kAmber);
}

}  // namespace app
