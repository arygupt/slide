#include "Hud.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "Draw.h"

namespace app {
namespace {

using namespace palette;
using slide::kCornerCount;

constexpr size_t kHistory = 600;      // 10 s at 60 Hz
constexpr double kSampleRate = 60.0;
constexpr double kTachMax = 8000.0;

std::string lapTime(double seconds) {
    if (seconds <= 0.0) return "-:--.---";
    char buf[32];
    const int minutes = static_cast<int>(seconds / 60.0);
    std::snprintf(buf, sizeof buf, "%d:%06.3f", minutes, seconds - 60.0 * minutes);
    return buf;
}

// raylib ring angles: degrees, clockwise on screen, 0 = +x.
float tachAngle(double rpm) { return static_cast<float>(135.0 + 270.0 * std::clamp(rpm, 0.0, kTachMax) / kTachMax); }

Vector2 polar(Vector2 c, float radius, float degrees) {
    const float a = degrees * DEG2RAD;
    return {c.x + radius * std::cos(a), c.y + radius * std::sin(a)};
}

void bar(Rectangle r, double fraction, Color fill) {
    DrawRectangleRec(r, withAlpha(WHITE, 0.06f));
    const float h = r.height * static_cast<float>(std::clamp(fraction, 0.0, 1.0));
    DrawRectangleRec({r.x, r.y + r.height - h, r.width, h}, fill);
}

void pill(const char* label, float x, float y, float s, bool enabled, bool active) {
    const float w = 46 * s, h = 20 * s;
    const Color edge = !enabled ? kFaint : active ? kAmber : kAccent;
    if (active) DrawRectangleRounded({x, y, w, h}, 0.4f, 6, withAlpha(kAmber, 0.85f));
    DrawRectangleRoundedLinesEx({x, y, w, h}, 0.4f, 6, 1.5f * s, edge);
    textCentered(label, x + 0.5f * w, y + 3 * s, 14 * s, active ? BLACK : enabled ? kText : kFaint);
    if (!enabled) DrawLineEx({x + 6 * s, y + 0.5f * h}, {x + w - 6 * s, y + 0.5f * h}, 1.5f * s, kFaint);
}

}  // namespace

Hud::Hud() : history_(kHistory) {}

float Hud::scale() const { return std::clamp(static_cast<float>(GetScreenHeight()) / 900.0f, 0.75f, 2.5f); }

const Hud::Snapshot& Hud::history(size_t ago) const {
    return history_[(head_ + kHistory - 1 - ago) % kHistory];
}

void Hud::clearHistory() { head_ = count_ = 0; }

void Hud::sample(const slide::Vehicle& v, double frameDt) {
    sampleClock_ += frameDt;
    if (sampleClock_ < 1.0 / kSampleRate) return;
    sampleClock_ = std::fmod(sampleClock_, 1.0 / kSampleRate);
    const slide::Vec2 a = v.localAcceleration();
    history_[head_] = {static_cast<float>(v.speed() * 3.6), static_cast<float>(v.engine().rpm()),
                       static_cast<float>(v.lastInput().throttle), static_cast<float>(v.lastInput().brake),
                       static_cast<float>(a.y / slide::kGravity), static_cast<float>(a.x / slide::kGravity)};
    head_ = (head_ + 1) % kHistory;
    count_ = std::min(count_ + 1, kHistory);
}

void Hud::draw(const slide::Vehicle& v, const FrameStats& stats, const HudState& state) const {
    const float s = scale();
    const float w = static_cast<float>(GetScreenWidth());
    const float h = static_cast<float>(GetScreenHeight());
    const float m = 14 * s;

    drawPerformance(stats, m, m);
    drawLap(state, 0.5f * w, m);
    drawTires(v, w - m, m);
    drawDash(v, 0.5f * w, h - m);
    drawGg(m, h - m);
    drawEngine(v, w - m, h - m);

    float badgeY = m + 92 * s;
    const auto badge = [&](const char* label, Color color) {
        const float bw = textWidth(label, 15 * s) + 20 * s;
        DrawRectangleRounded({0.5f * w - 0.5f * bw, badgeY, bw, 24 * s}, 0.5f, 6, withAlpha(color, 0.9f));
        textCentered(label, 0.5f * w, badgeY + 4 * s, 15 * s, BLACK);
        badgeY += 30 * s;
    };
    if (state.autopilot) badge("AUTOPILOT", kAccent);
    if (state.recording) badge("REC  telemetry", kRed);

    if (!v.engine().running()) {
        textCentered("ENGINE STALLED", 0.5f * w, 0.36f * h, 40 * s, kRed);
        textCentered("press I to crank", 0.5f * w, 0.36f * h + 46 * s, 18 * s, kText);
    }
    if (state.toastAge < 2.5 && !state.toast.empty()) {
        const float alpha = static_cast<float>(std::min(1.0, (2.5 - state.toastAge) * 2.0));
        const float tw = textWidth(state.toast.c_str(), 18 * s) + 28 * s;
        const Rectangle r{0.5f * w - 0.5f * tw, 0.24f * h, tw, 34 * s};
        DrawRectangleRounded(r, 0.4f, 6, withAlpha(kPanel, 0.9f * alpha));
        textCentered(state.toast.c_str(), 0.5f * w, r.y + 7 * s, 18 * s, withAlpha(kText, alpha));
    }
    if (state.help) drawHelp();
}

void Hud::drawPerformance(const FrameStats& st, float x, float y) const {
    const float s = scale();
    panel({x, y, 300 * s, 118 * s});
    char buf[128];
    text("SLIDE", x + 14 * s, y + 10 * s, 22 * s, kAccent);
    text("vehicle dynamics", x + 92 * s, y + 15 * s, 14 * s, kDim);

    std::snprintf(buf, sizeof buf, "%.0f", st.fps);
    text(buf, x + 14 * s, y + 38 * s, 34 * s, kText);
    text("fps", x + 18 * s + textWidth(buf, 34 * s), y + 52 * s, 15 * s, kDim);
    std::snprintf(buf, sizeof buf, "%.2f ms/frame", st.frameMs);
    textRight(buf, x + 286 * s, y + 52 * s, 15 * s, kDim);

    const double physicsMs = st.stepMicros * st.stepsPerFrame / 1000.0;
    std::snprintf(buf, sizeof buf, "physics %.0f Hz  %.2f us/step", st.physicsHz, st.stepMicros);
    text(buf, x + 14 * s, y + 78 * s, 15 * s, kText);
    std::snprintf(buf, sizeof buf, "%.1f steps/frame  %.1f%% of frame", st.stepsPerFrame,
                  st.frameMs > 0.0 ? 100.0 * physicsMs / st.frameMs : 0.0);
    text(buf, x + 14 * s, y + 97 * s, 14 * s, kDim);
}

void Hud::drawLap(const HudState& state, float cx, float y) const {
    const float s = scale();
    const float w = 250 * s;
    panel({cx - 0.5f * w, y, w, 78 * s});
    char buf[64];
    std::snprintf(buf, sizeof buf, "LAP %d", state.laps + 1);
    text(buf, cx - 0.5f * w + 14 * s, y + 10 * s, 14 * s, kDim);
    textCentered(lapTime(state.currentLap).c_str(), cx, y + 22 * s, 32 * s, kText);
    std::snprintf(buf, sizeof buf, "last %s   best %s", lapTime(state.lastLap).c_str(),
                  lapTime(state.bestLap).c_str());
    textCentered(buf, cx, y + 56 * s, 14 * s, kDim);
}

void Hud::drawTires(const slide::Vehicle& v, float right, float y) const {
    const float s = scale();
    const float cw = 160 * s, ch = 150 * s, pad = 8 * s;
    const float pw = 2 * cw + 3 * pad, ph = 2 * ch + 3 * pad + 26 * s;
    const float px = right - pw;
    panel({px, y, pw, ph});
    static constexpr const char* kLabels[] = {"FL", "FR", "RL", "RR"};
    char buf[64];

    for (int i = 0; i < kCornerCount; ++i) {
        const slide::Wheel& w = v.wheel(i);
        const auto& corner = v.suspension().corner(i);
        const float x0 = px + pad + static_cast<float>(i % 2) * (cw + pad);
        const float y0 = y + pad + static_cast<float>(i / 2) * (ch + pad);
        DrawRectangleRounded({x0, y0, cw, ch}, 0.08f, 6, withAlpha(WHITE, 0.03f));
        text(kLabels[i], x0 + 8 * s, y0 + 6 * s, 15 * s, kDim);
        const char* surface = w.surface.kind == slide::SurfaceKind::Grass  ? "grass"
                              : w.surface.kind == slide::SurfaceKind::Curb ? "curb"
                                                                           : "";
        text(surface, x0 + 32 * s, y0 + 7 * s, 13 * s, kAmber);
        if (v.aids().brakeModulation(i) < 0.99) text("ABS", x0 + cw - 34 * s, y0 + 6 * s, 14 * s, kAmber);

        // Friction circle: lateral force across, longitudinal force up.
        const Vector2 c{x0 + 52 * s, y0 + 72 * s};
        const float r = 40 * s;
        DrawRing(c, r - 1.5f * s, r, 0, 360, 48, withAlpha(WHITE, 0.25f));
        DrawRing(c, 0.5f * r - 1.0f * s, 0.5f * r, 0, 360, 36, withAlpha(WHITE, 0.08f));
        DrawLineEx({c.x - r, c.y}, {c.x + r, c.y}, 1.0f, withAlpha(WHITE, 0.08f));
        DrawLineEx({c.x, c.y - r}, {c.x, c.y + r}, 1.0f, withAlpha(WHITE, 0.08f));
        if (w.tire.gripLimit > 0.0) {
            const float fx = static_cast<float>(std::clamp(w.tire.Fx / w.tire.gripLimit, -1.2, 1.2));
            const float fy = static_cast<float>(std::clamp(w.tire.Fy / w.tire.gripLimit, -1.2, 1.2));
            const Vector2 dot{c.x - fy * r, c.y - fx * r};
            const Color col = heat(w.tire.utilization);
            DrawLineEx(c, dot, 2.0f * s, withAlpha(col, 0.6f));
            DrawCircleV(dot, 5.5f * s, col);
        }

        // Normal load and suspension travel bars.
        const double staticLoad = corner.staticLoad + 400.0;
        bar({x0 + 106 * s, y0 + 30 * s, 10 * s, 84 * s}, corner.tireLoad / (2.4 * staticLoad), kBlue);
        const Rectangle travel{x0 + 126 * s, y0 + 30 * s, 10 * s, 84 * s};
        DrawRectangleRec(travel, withAlpha(WHITE, 0.06f));
        const float mid = travel.y + 0.5f * travel.height;
        const float off = static_cast<float>(std::clamp(corner.compression / 0.09, -1.0, 1.0)) * 0.5f * travel.height;
        DrawRectangleRec({travel.x, std::min(mid, mid - off), travel.width, std::abs(off)}, kViolet);
        DrawLineEx({travel.x - 2 * s, mid}, {travel.x + travel.width + 2 * s, mid}, 1.0f, kDim);
        std::snprintf(buf, sizeof buf, "%.1fkN", corner.tireLoad / 1000.0);
        textCentered(buf, x0 + 121 * s, y0 + 116 * s, 12 * s, kDim);

        std::snprintf(buf, sizeof buf, "SR %+.2f  SA %+.1f%s", w.tire.slipRatio, slide::toDeg(w.tire.slipAngle),
                      "\xC2\xB0");
        text(buf, x0 + 8 * s, y0 + ch - 20 * s, 13 * s, kText);
    }

    const auto& body = v.suspension().body();
    std::snprintf(buf, sizeof buf, "pitch %+.2f\xC2\xB0  roll %+.2f\xC2\xB0  heave %+.0f mm", slide::toDeg(body.pitch),
                  slide::toDeg(body.roll), body.heave * 1000.0);
    text(buf, px + pad + 4 * s, y + ph - 24 * s, 14 * s, kDim);
}

void Hud::drawDash(const slide::Vehicle& v, float cx, float bottom) const {
    const float s = scale();
    const float w = 580 * s, h = 182 * s;
    const float px = cx - 0.5f * w, py = bottom - h;
    panel({px, py, w, h});
    char buf[64];

    // Tachometer.
    const Vector2 c{px + 104 * s, py + 100 * s};
    const float r = 76 * s;
    const double rpm = v.engine().rpm();
    const double redline = v.params().engine.redlineRpm;
    DrawRing(c, r - 9 * s, r, tachAngle(0), tachAngle(kTachMax), 64, withAlpha(WHITE, 0.07f));
    DrawRing(c, r - 9 * s, r, tachAngle(redline), tachAngle(kTachMax), 16, withAlpha(kRed, 0.55f));
    const Color fill = rpm > redline - 400.0 ? kRed : rpm > 6000.0 ? kAmber : kAccent;
    DrawRing(c, r - 9 * s, r, tachAngle(0), tachAngle(rpm), 64, fill);
    for (int k = 0; k <= 8; ++k) {
        const float a = tachAngle(k * 1000.0);
        DrawLineEx(polar(c, r - 14 * s, a), polar(c, r - 20 * s, a), 2.0f * s, kDim);
        std::snprintf(buf, sizeof buf, "%d", k);
        const Vector2 p = polar(c, r - 31 * s, a);
        textCentered(buf, p.x, p.y - 7 * s, 13 * s, k >= 8 ? kRed : kDim);
    }
    DrawLineEx(c, polar(c, r - 4 * s, tachAngle(rpm)), 3.0f * s, RAYWHITE);
    DrawCircleV(c, 5 * s, RAYWHITE);

    const int gear = v.drivetrain().gear();
    std::snprintf(buf, sizeof buf, "%s", gear < 0 ? "R" : gear == 0 ? "N" : std::to_string(gear).c_str());
    textCentered(buf, c.x, c.y + 18 * s, 34 * s, kText);
    std::snprintf(buf, sizeof buf, "%.0f rpm", rpm);
    textCentered(buf, c.x, c.y + 56 * s, 14 * s, kDim);

    // Shift lights.
    for (int k = 0; k < 7; ++k) {
        const double threshold = 5800.0 + k * 230.0;
        const bool lit = rpm >= threshold;
        const bool flash = rpm >= 7300.0 && std::fmod(GetTime(), 0.12) < 0.06;
        const Color col = k < 3 ? kAccent : k < 5 ? kAmber : kRed;
        DrawCircleV({px + 30 * s + k * 25 * s, py + 14 * s}, 6 * s,
                    flash ? kBlue : lit ? col : withAlpha(WHITE, 0.07f));
    }

    // Speed and drivetrain.
    const float mx = px + 290 * s;
    std::snprintf(buf, sizeof buf, "%.0f", v.speed() * 3.6);
    textCentered(buf, mx, py + 30 * s, 58 * s, kText);
    textCentered("km/h", mx, py + 90 * s, 15 * s, kDim);
    const bool manual = v.drivetrain().mode() == slide::Drivetrain::Mode::Manual;
    std::snprintf(buf, sizeof buf, "%s  clutch %3.0f%%", manual ? "MANUAL" : "AUTO",
                  100.0 * v.drivetrain().clutchEngagement());
    textCentered(buf, mx, py + 114 * s, 14 * s, kDim);

    const auto& aids = v.aids();
    const auto& ap = aids.params();
    pill("ABS", mx - 80 * s, py + 144 * s, s, ap.absEnabled, aids.absActive());
    pill("TCS", mx - 23 * s, py + 144 * s, s, ap.tcsEnabled, aids.tcsActive());
    pill("ESC", mx + 34 * s, py + 144 * s, s, ap.escEnabled, aids.escActive());

    // Pedals and steering.
    const slide::DriverInput& in = v.lastInput();
    const float bx = px + 410 * s, by = py + 22 * s, bh = 108 * s, bw = 22 * s;
    const struct { const char* label; double value; Color color; } pedals[] = {
        {"THR", in.throttle, kAccent}, {"BRK", in.brake, kRed}, {"CLU", v.drivetrain().clutchEngagement() < 0.99 ? 1.0 - v.drivetrain().clutchEngagement() : 0.0, kBlue}};
    for (int k = 0; k < 3; ++k) {
        const float x = bx + k * 42 * s;
        bar({x, by, bw, bh}, pedals[k].value, pedals[k].color);
        textCentered(pedals[k].label, x + 0.5f * bw, by + bh + 6 * s, 12 * s, kDim);
    }
    if (in.handbrake) text("HB", bx + 128 * s, by + 4 * s, 15 * s, kRed);
    const Rectangle steer{bx, py + 160 * s, 148 * s, 8 * s};
    DrawRectangleRec(steer, withAlpha(WHITE, 0.06f));
    const float steerFrac = static_cast<float>(v.steerAngle() / v.params().steering.maxAngle);
    const float sx = steer.x + 0.5f * steer.width;
    DrawRectangleRec({std::min(sx, sx - steerFrac * 0.5f * steer.width), steer.y,
                      std::abs(steerFrac) * 0.5f * steer.width, steer.height},
                     kText);
    DrawLineEx({sx, steer.y - 3 * s}, {sx, steer.y + steer.height + 3 * s}, 1.0f, kDim);
}

void Hud::drawGg(float x, float bottom) const {
    const float s = scale();
    const float size = 220 * s;
    const float y = bottom - size;
    panel({x, y, size, size});
    text("G-G", x + 12 * s, y + 8 * s, 14 * s, kDim);
    const Vector2 c{x + 0.5f * size, y + 0.5f * size + 6 * s};
    const float perG = 62 * s;
    for (float g : {0.5f, 1.0f, 1.5f}) {
        DrawRing(c, g * perG - 1.0f, g * perG, 0, 360, 48, withAlpha(WHITE, g == 1.0f ? 0.22f : 0.08f));
    }
    DrawLineEx({c.x - 1.5f * perG, c.y}, {c.x + 1.5f * perG, c.y}, 1.0f, withAlpha(WHITE, 0.1f));
    DrawLineEx({c.x, c.y - 1.5f * perG}, {c.x, c.y + 1.5f * perG}, 1.0f, withAlpha(WHITE, 0.1f));
    textRight("1g", c.x + perG - 2 * s, c.y - perG - 2 * s, 12 * s, kFaint);
    if (count_ == 0) return;

    const size_t trail = std::min<size_t>(count_, 120);
    for (size_t k = trail; k-- > 1;) {
        const Snapshot& p = history(k);
        const float a = 1.0f - static_cast<float>(k) / static_cast<float>(trail);
        DrawCircleV({c.x - p.lateralG * perG, c.y - p.longitudinalG * perG}, 2.5f * s, withAlpha(kBlue, 0.6f * a));
    }
    const Snapshot& now = history(0);
    DrawCircleV({c.x - now.lateralG * perG, c.y - now.longitudinalG * perG}, 6 * s, kText);
    char buf[48];
    std::snprintf(buf, sizeof buf, "%.2f g", std::hypot(now.lateralG, now.longitudinalG));
    textRight(buf, x + size - 12 * s, y + 8 * s, 15 * s, kText);
}

void Hud::drawEngine(const slide::Vehicle& v, float right, float bottom) const {
    const float s = scale();
    const float w = 400 * s, h = 220 * s;
    const float x = right - w, y = bottom - h;
    panel({x, y, w, h});
    const slide::Engine::Readout& e = v.engine().readout();
    const auto& d = v.drivetrain();
    char buf[96];

    const auto row = [&](int line, int column, const char* label, const char* value) {
        const float cx = x + 14 * s + column * 192 * s;
        const float cy = y + 10 * s + line * 20 * s;
        text(label, cx, cy, 14 * s, kDim);
        textRight(value, cx + 176 * s, cy, 14 * s, kText);
    };
    std::snprintf(buf, sizeof buf, "%.0f Nm", e.netTorque);
    row(0, 0, "torque", buf);
    std::snprintf(buf, sizeof buf, "%.0f kW", e.netTorque * v.engine().omega() / 1000.0);
    row(0, 1, "power", buf);
    std::snprintf(buf, sizeof buf, "%.1f kPa", e.manifoldPressure / 1000.0);
    row(1, 0, "manifold", buf);
    std::snprintf(buf, sizeof buf, "%.1f g/s", e.cylinderAirflow * 1000.0);
    row(1, 1, "airflow", buf);
    std::snprintf(buf, sizeof buf, "%.1f\xC2\xB0", slide::toDeg(e.throttleAngle));
    row(2, 0, "throttle plate", buf);
    std::snprintf(buf, sizeof buf, "%s%.2f g/s", e.fuelCut ? "CUT " : "", e.fuelFlow * 1000.0);
    row(2, 1, "fuel", buf);
    std::snprintf(buf, sizeof buf, "%.0f rpm", d.clutchSlip() * slide::kRadToRpm);
    row(3, 0, "clutch slip", buf);
    std::snprintf(buf, sizeof buf, "%.0f Nm", d.lsdTorque());
    row(3, 1, "LSD bias", buf);

    // Strip chart, last 10 s.
    const Rectangle chart{x + 14 * s, y + 96 * s, w - 28 * s, h - 118 * s};
    DrawRectangleRec(chart, withAlpha(WHITE, 0.03f));
    for (int k = 1; k < 4; ++k) {
        const float gy = chart.y + chart.height * k / 4.0f;
        DrawLineEx({chart.x, gy}, {chart.x + chart.width, gy}, 1.0f, withAlpha(WHITE, 0.05f));
    }
    if (count_ > 1) {
        static Vector2 points[kHistory];
        const auto trace = [&](float (*value)(const Snapshot&), Color color) {
            const size_t n = count_;
            for (size_t k = 0; k < n; ++k) {
                const Snapshot& p = history(n - 1 - k);
                const float t = static_cast<float>(k + kHistory - n) / static_cast<float>(kHistory - 1);
                const float val = std::clamp(value(p), 0.0f, 1.0f);
                points[k] = {chart.x + t * chart.width, chart.y + chart.height * (1.0f - val)};
            }
            DrawSplineLinear(points, static_cast<int>(n), 1.6f * s, color);
        };
        trace([](const Snapshot& p) { return p.throttle * 0.96f + 0.02f; }, withAlpha(kAccent, 0.8f));
        trace([](const Snapshot& p) { return p.brake * 0.96f + 0.02f; }, withAlpha(kRed, 0.8f));
        trace([](const Snapshot& p) { return p.rpm / 8000.0f; }, withAlpha(kAmber, 0.9f));
        trace([](const Snapshot& p) { return p.speed / 250.0f; }, kText);
    }
    const float ly = chart.y + chart.height + 5 * s;
    text("speed", chart.x, ly, 12 * s, kText);
    text("rpm", chart.x + 50 * s, ly, 12 * s, kAmber);
    text("throttle", chart.x + 84 * s, ly, 12 * s, kAccent);
    text("brake", chart.x + 146 * s, ly, 12 * s, kRed);
    textRight("10 s", chart.x + chart.width, ly, 12 * s, kFaint);
}

void Hud::drawHelp() const {
    const float s = scale();
    const float w = 520 * s, h = 452 * s;
    const float x = 0.5f * (static_cast<float>(GetScreenWidth()) - w);
    const float y = 0.5f * (static_cast<float>(GetScreenHeight()) - h);
    DrawRectangleRounded({x, y, w, h}, 0.04f, 8, withAlpha(kPanel, 0.96f));
    DrawRectangleRoundedLinesEx({x, y, w, h}, 0.04f, 8, 1.0f, kPanelEdge);
    text("Controls", x + 22 * s, y + 18 * s, 22 * s, kAccent);
    static constexpr const char* kKeys[][2] = {
        {"W / S  or  Up / Down", "throttle / brake (S reverses when stopped)"},
        {"A / D  or  Left / Right", "steer"},
        {"Space", "handbrake"},
        {"Left Shift", "clutch"},
        {"E / Q", "shift up / down (switches to manual)"},
        {"M", "automatic / manual gearbox"},
        {"1  2  3", "toggle ABS, traction control, stability control"},
        {"P", "autopilot"},
        {"T", "record telemetry to CSV"},
        {"V", "x-ray: tire forces and utilization"},
        {"C", "chase / north-up camera"},
        {"Mouse wheel  or  + / -", "zoom"},
        {"R", "reset to the start line"},
        {"I", "crank the engine"},
        {"Tab", "hide HUD"},
        {"F1 / H", "this help"},
    };
    float ly = y + 58 * s;
    for (const auto& k : kKeys) {
        text(k[0], x + 22 * s, ly, 15 * s, kText);
        text(k[1], x + 230 * s, ly, 15 * s, kDim);
        ly += 23 * s;
    }
}

}  // namespace app
