#pragma once

// Shared rendering helpers. World space is meters with y up; raylib's 2D
// space has y down, so every world point goes through rl() on its way out.

#include <algorithm>
#include <cmath>

#include "raylib.h"
#include "slide/Math.h"

namespace app {

inline Vector2 rl(slide::Vec2 p) { return {static_cast<float>(p.x), static_cast<float>(-p.y)}; }

// raylib rotations are clockwise degrees on a y-down screen.
inline float rlAngle(double worldRadians) { return static_cast<float>(-slide::toDeg(worldRadians)); }

inline Color withAlpha(Color c, float alpha) {
    c.a = static_cast<unsigned char>(std::clamp(alpha, 0.0f, 1.0f) * 255.0f);
    return c;
}

inline Color mix(Color a, Color b, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    const auto ch = [t](unsigned char x, unsigned char y) {
        return static_cast<unsigned char>(x + (y - x) * t);
    };
    return {ch(a.r, b.r), ch(a.g, b.g), ch(a.b, b.b), ch(a.a, b.a)};
}

namespace palette {
inline constexpr Color kPanel{12, 15, 20, 205};
inline constexpr Color kPanelEdge{255, 255, 255, 22};
inline constexpr Color kText{228, 232, 238, 255};
inline constexpr Color kDim{132, 140, 152, 255};
inline constexpr Color kFaint{80, 88, 100, 255};
inline constexpr Color kAccent{52, 211, 153, 255};  // emerald
inline constexpr Color kAmber{245, 158, 11, 255};
inline constexpr Color kRed{239, 68, 68, 255};
inline constexpr Color kBlue{96, 165, 250, 255};
inline constexpr Color kViolet{167, 139, 250, 255};
}  // namespace palette

// Green -> amber -> red for a 0..1 utilization.
inline Color heat(double t) {
    const float f = static_cast<float>(std::clamp(t, 0.0, 1.2));
    if (f < 0.75f) return mix(palette::kAccent, palette::kAmber, f / 0.75f);
    return mix(palette::kAmber, palette::kRed, (f - 0.75f) / 0.3f);
}

// One UI font for the whole app, loaded once in main().
inline Font& uiFont() {
    static Font font = GetFontDefault();
    return font;
}

inline void text(const char* s, float x, float y, float size, Color color) {
    DrawTextEx(uiFont(), s, {std::round(x), std::round(y)}, size, 0.0f, color);
}

inline float textWidth(const char* s, float size) { return MeasureTextEx(uiFont(), s, size, 0.0f).x; }

inline void textRight(const char* s, float right, float y, float size, Color color) {
    text(s, right - textWidth(s, size), y, size, color);
}

inline void textCentered(const char* s, float cx, float y, float size, Color color) {
    text(s, cx - 0.5f * textWidth(s, size), y, size, color);
}

inline void panel(Rectangle r) {
    DrawRectangleRounded(r, 0.06f, 6, palette::kPanel);
    DrawRectangleRoundedLinesEx(r, 0.06f, 6, 1.0f, palette::kPanelEdge);
}

}  // namespace app
