#pragma once

// Small, dependency-free math kit for the simulation core. Everything is SI:
// meters, seconds, kilograms, newtons, radians.

#include <algorithm>
#include <cmath>
#include <numbers>

namespace slide {

inline constexpr double kPi = std::numbers::pi;
inline constexpr double kTwoPi = 2.0 * std::numbers::pi;
inline constexpr double kGravity = 9.80665;
inline constexpr double kRadToRpm = 60.0 / kTwoPi;
inline constexpr double kRpmToRad = kTwoPi / 60.0;

constexpr double deg(double degrees) { return degrees * kPi / 180.0; }
constexpr double toDeg(double radians) { return radians * 180.0 / kPi; }
constexpr double sq(double x) { return x * x; }
constexpr double lerp(double a, double b, double t) { return a + (b - a) * t; }

constexpr double sign(double x) { return (x > 0.0) - (x < 0.0); }

constexpr double smoothstep(double edge0, double edge1, double x) {
    const double t = std::clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// Moves `current` toward `target` by at most `maxDelta`.
constexpr double approach(double current, double target, double maxDelta) {
    if (current < target) return std::min(current + maxDelta, target);
    return std::max(current - maxDelta, target);
}

// Wraps an angle to (-pi, pi].
inline double wrapAngle(double a) { return std::remainder(a, kTwoPi); }

struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    constexpr Vec2() = default;
    constexpr Vec2(double x_, double y_) : x(x_), y(y_) {}

    constexpr Vec2 operator+(Vec2 o) const { return {x + o.x, y + o.y}; }
    constexpr Vec2 operator-(Vec2 o) const { return {x - o.x, y - o.y}; }
    constexpr Vec2 operator-() const { return {-x, -y}; }
    constexpr Vec2 operator*(double s) const { return {x * s, y * s}; }
    constexpr Vec2 operator/(double s) const { return {x / s, y / s}; }
    constexpr Vec2& operator+=(Vec2 o) { x += o.x; y += o.y; return *this; }
    constexpr Vec2& operator-=(Vec2 o) { x -= o.x; y -= o.y; return *this; }
    constexpr Vec2& operator*=(double s) { x *= s; y *= s; return *this; }
    constexpr bool operator==(const Vec2&) const = default;

    double length() const { return std::hypot(x, y); }
    constexpr double lengthSq() const { return x * x + y * y; }
    Vec2 normalized() const {
        const double len = length();
        return len > 1e-12 ? Vec2{x / len, y / len} : Vec2{};
    }
    // Rotates counter-clockwise by the angle whose cosine/sine are given.
    constexpr Vec2 rotated(double c, double s) const { return {c * x - s * y, s * x + c * y}; }
    Vec2 rotated(double angle) const { return rotated(std::cos(angle), std::sin(angle)); }
    constexpr Vec2 perp() const { return {-y, x}; }  // +90 degrees
};

constexpr Vec2 operator*(double s, Vec2 v) { return v * s; }
constexpr double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
constexpr double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }

// A planar rotation with cached sine/cosine, used to move vectors between the
// world frame and the vehicle body frame (x forward, y left).
struct Rot2 {
    double c = 1.0;
    double s = 0.0;

    Rot2() = default;
    explicit Rot2(double angle) : c(std::cos(angle)), s(std::sin(angle)) {}

    constexpr Vec2 apply(Vec2 v) const { return v.rotated(c, s); }
    constexpr Vec2 applyInverse(Vec2 v) const { return v.rotated(c, -s); }
};

}  // namespace slide
