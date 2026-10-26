#include "slide/Track.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace slide {
namespace {

constexpr double kCurbCurvature = 1.0 / 120.0;  // corners tighter than this get curbs
constexpr double kCurbHeight = 0.012;           // [m]
constexpr double kCurbPeriod = 1.0;             // rumble strip pitch [m]
constexpr double kEdgeBlend = 0.3;              // curb ramp width [m]
constexpr double kGrassBlend = 1.5;             // grass bumps fade in over this [m]
constexpr int kSplineSubdivisions = 64;

// Centripetal Catmull-Rom (Barry-Goldman form); unlike the uniform variant it
// cannot form cusps or loops between unevenly spaced control points.
Vec2 catmullRom(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3, double u) {
    const auto knot = [](double t, Vec2 a, Vec2 b) { return t + std::sqrt((b - a).length()); };
    const double t0 = 0.0;
    const double t1 = knot(t0, p0, p1);
    const double t2 = knot(t1, p1, p2);
    const double t3 = knot(t2, p2, p3);
    const double t = lerp(t1, t2, u);
    const Vec2 a1 = p0 * ((t1 - t) / (t1 - t0)) + p1 * ((t - t0) / (t1 - t0));
    const Vec2 a2 = p1 * ((t2 - t) / (t2 - t1)) + p2 * ((t - t1) / (t2 - t1));
    const Vec2 a3 = p2 * ((t3 - t) / (t3 - t2)) + p3 * ((t - t2) / (t3 - t2));
    const Vec2 b1 = a1 * ((t2 - t) / (t2 - t0)) + a2 * ((t - t0) / (t2 - t0));
    const Vec2 b2 = a2 * ((t3 - t) / (t3 - t1)) + a3 * ((t - t1) / (t3 - t1));
    return b1 * ((t2 - t) / (t2 - t1)) + b2 * ((t - t1) / (t2 - t1));
}

double grassHeight(Vec2 p) {
    return 0.018 * std::sin(0.83 * p.x + 0.4) * std::sin(0.71 * p.y + 1.3) +
           0.008 * std::sin(2.1 * p.x - 1.7 * p.y);
}

}  // namespace

Track::Track(const std::vector<Vec2>& controlPoints, double halfWidth, double spacing)
    : halfWidth_(halfWidth), spacing_(spacing) {
    // Dense polyline through the closed spline.
    const size_t n = controlPoints.size();
    std::vector<Vec2> dense;
    dense.reserve(n * kSplineSubdivisions + 1);
    for (size_t i = 0; i < n; ++i) {
        const Vec2 p0 = controlPoints[(i + n - 1) % n];
        const Vec2 p1 = controlPoints[i];
        const Vec2 p2 = controlPoints[(i + 1) % n];
        const Vec2 p3 = controlPoints[(i + 2) % n];
        for (int k = 0; k < kSplineSubdivisions; ++k) {
            dense.push_back(catmullRom(p0, p1, p2, p3, static_cast<double>(k) / kSplineSubdivisions));
        }
    }
    dense.push_back(dense.front());

    std::vector<double> arc(dense.size(), 0.0);
    for (size_t i = 1; i < dense.size(); ++i) arc[i] = arc[i - 1] + (dense[i] - dense[i - 1]).length();
    length_ = arc.back();

    // Resample at even spacing so lookups by distance are O(1).
    const int count = std::max(8, static_cast<int>(std::round(length_ / spacing)));
    spacing_ = length_ / count;
    samples_.resize(static_cast<size_t>(count));
    size_t seg = 0;
    for (int i = 0; i < count; ++i) {
        const double s = i * spacing_;
        while (seg + 2 < arc.size() && arc[seg + 1] < s) ++seg;
        const double t = (s - arc[seg]) / std::max(1e-9, arc[seg + 1] - arc[seg]);
        samples_[static_cast<size_t>(i)].point = dense[seg] + (dense[seg + 1] - dense[seg]) * t;
        samples_[static_cast<size_t>(i)].distance = s;
    }

    const auto at = [&](int i) -> Sample& { return samples_[static_cast<size_t>((i + count) % count)]; };
    for (int i = 0; i < count; ++i) at(i).tangent = (at(i + 1).point - at(i - 1).point).normalized();
    std::vector<double> rawCurvature(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        const double turn = wrapAngle(std::atan2(at(i + 1).tangent.y, at(i + 1).tangent.x) -
                                      std::atan2(at(i - 1).tangent.y, at(i - 1).tangent.x));
        rawCurvature[static_cast<size_t>(i)] = turn / (2.0 * spacing_);
    }
    // Light smoothing; finite differences on a resampled spline are noisy.
    for (int i = 0; i < count; ++i) {
        double sum = 0.0;
        for (int k = -3; k <= 3; ++k) sum += rawCurvature[static_cast<size_t>((i + k + count) % count)];
        at(i).curvature = sum / 7.0;
    }

    buildGrid();
}

Track Track::circuit() {
    return Track({{-150, 0},   {0, 0},       {150, 0},     {230, 15},    {275, 60},
                  {285, 120},  {260, 170},   {210, 190},   {160, 175},   {130, 140},
                  {100, 125},  {60, 135},    {30, 170},    {20, 220},    {-10, 260},
                  {-60, 270},  {-120, 250},  {-150, 210},  {-140, 160},  {-100, 130},
                  {-95, 95},   {-130, 70},   {-190, 60},   {-215, 30},   {-200, 5}});
}

void Track::buildGrid() {
    const double margin = halfWidth_ + curbWidth_ + kGrassBlend + 4.0;
    Vec2 lo{std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    Vec2 hi = -lo;
    for (const Sample& s : samples_) {
        lo = {std::min(lo.x, s.point.x), std::min(lo.y, s.point.y)};
        hi = {std::max(hi.x, s.point.x), std::max(hi.y, s.point.y)};
    }
    gridOrigin_ = lo - Vec2{margin, margin};
    gridWidth_ = static_cast<int>(std::ceil((hi.x - lo.x + 2.0 * margin) / cellSize_));
    gridHeight_ = static_cast<int>(std::ceil((hi.y - lo.y + 2.0 * margin) / cellSize_));

    std::vector<std::vector<int>> cells(static_cast<size_t>(gridWidth_ * gridHeight_));
    const int count = static_cast<int>(samples_.size());
    for (int i = 0; i < count; ++i) {
        const Vec2 a = samples_[static_cast<size_t>(i)].point;
        const Vec2 b = samples_[static_cast<size_t>((i + 1) % count)].point;
        const int x0 = static_cast<int>((std::min(a.x, b.x) - margin - gridOrigin_.x) / cellSize_);
        const int x1 = static_cast<int>((std::max(a.x, b.x) + margin - gridOrigin_.x) / cellSize_);
        const int y0 = static_cast<int>((std::min(a.y, b.y) - margin - gridOrigin_.y) / cellSize_);
        const int y1 = static_cast<int>((std::max(a.y, b.y) + margin - gridOrigin_.y) / cellSize_);
        for (int cy = std::max(0, y0); cy <= std::min(gridHeight_ - 1, y1); ++cy) {
            for (int cx = std::max(0, x0); cx <= std::min(gridWidth_ - 1, x1); ++cx) {
                cells[static_cast<size_t>(cellIndex(cx, cy))].push_back(i);
            }
        }
    }
    cellStart_.assign(cells.size() + 1, 0);
    gridSegments_.clear();
    for (size_t c = 0; c < cells.size(); ++c) {
        cellStart_[c] = static_cast<int>(gridSegments_.size());
        gridSegments_.insert(gridSegments_.end(), cells[c].begin(), cells[c].end());
    }
    cellStart_.back() = static_cast<int>(gridSegments_.size());
}

Track::Projection Track::project(Vec2 p) const {
    Projection out;
    out.lateral = std::numeric_limits<double>::infinity();
    const int cx = static_cast<int>(std::floor((p.x - gridOrigin_.x) / cellSize_));
    const int cy = static_cast<int>(std::floor((p.y - gridOrigin_.y) / cellSize_));
    if (cx < 0 || cy < 0 || cx >= gridWidth_ || cy >= gridHeight_) return out;

    const int cell = cellIndex(cx, cy);
    const int count = static_cast<int>(samples_.size());
    double best = std::numeric_limits<double>::max();
    for (int k = cellStart_[static_cast<size_t>(cell)]; k < cellStart_[static_cast<size_t>(cell) + 1]; ++k) {
        const int i = gridSegments_[static_cast<size_t>(k)];
        const Vec2 a = samples_[static_cast<size_t>(i)].point;
        const Vec2 ab = samples_[static_cast<size_t>((i + 1) % count)].point - a;
        const double t = std::clamp(dot(p - a, ab) / ab.lengthSq(), 0.0, 1.0);
        const Vec2 offset = p - (a + ab * t);
        const double d2 = offset.lengthSq();
        if (d2 < best) {
            best = d2;
            out.distance = samples_[static_cast<size_t>(i)].distance + t * spacing_;
            out.lateral = std::copysign(std::sqrt(d2), cross(ab, offset));
            out.nearTrack = true;
        }
    }
    return out;
}

bool Track::hasCurb(double distance) const {
    return std::abs(curvatureAt(distance)) > kCurbCurvature;
}

SurfaceSample Track::sample(Vec2 p) const {
    const Projection proj = project(p);
    const double lateral = std::abs(proj.lateral);
    if (lateral <= halfWidth_) return {};

    const bool curb = proj.nearTrack && hasCurb(proj.distance);
    const double curbEdge = halfWidth_ + curbWidth_;
    if (curb && lateral <= curbEdge) {
        const double ramp = smoothstep(halfWidth_, halfWidth_ + kEdgeBlend, lateral) *
                            (1.0 - smoothstep(curbEdge - kEdgeBlend, curbEdge, lateral));
        const double rumble = 0.5 - 0.5 * std::cos(kTwoPi * proj.distance / kCurbPeriod);
        return {kCurbHeight * ramp * rumble, 0.95, 0.016, SurfaceKind::Curb};
    }
    const double edge = curb ? curbEdge : halfWidth_;
    const double fade = std::isinf(lateral) ? 1.0 : smoothstep(edge, edge + kGrassBlend, lateral);
    return {grassHeight(p) * fade, 0.55, 0.05, SurfaceKind::Grass};
}

double Track::wrap(double distance) const {
    const double s = std::fmod(distance, length_);
    return s < 0.0 ? s + length_ : s;
}

Vec2 Track::pointAt(double distance) const {
    const double s = wrap(distance) / spacing_;
    const size_t count = samples_.size();
    const size_t i = static_cast<size_t>(s) % count;
    const double t = s - std::floor(s);
    return samples_[i].point + (samples_[(i + 1) % count].point - samples_[i].point) * t;
}

Vec2 Track::tangentAt(double distance) const {
    const double s = wrap(distance) / spacing_;
    const size_t count = samples_.size();
    const size_t i = static_cast<size_t>(s) % count;
    const double t = s - std::floor(s);
    return (samples_[i].tangent * (1.0 - t) + samples_[(i + 1) % count].tangent * t).normalized();
}

double Track::curvatureAt(double distance) const {
    const double s = wrap(distance) / spacing_;
    const size_t count = samples_.size();
    const size_t i = static_cast<size_t>(s) % count;
    const double t = s - std::floor(s);
    return lerp(samples_[i].curvature, samples_[(i + 1) % count].curvature, t);
}

}  // namespace slide
