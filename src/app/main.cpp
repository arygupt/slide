// Slide visualizer: drives the physics core at a fixed rate (1 kHz by default)
// and renders as fast as the GPU allows, interpolating the car pose between
// physics steps so motion stays smooth at any frame rate.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>

#include "Draw.h"
#include "Effects.h"
#include "Hud.h"
#include "WorldRenderer.h"
#include "raylib.h"
#include "rlgl.h"
#include "slide/Autopilot.h"
#include "slide/Telemetry.h"
#include "slide/Track.h"
#include "slide/Vehicle.h"

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#endif

namespace {

using namespace slide;
using Clock = std::chrono::steady_clock;

struct Args {
    int width = 1600;
    int height = 900;
    double hz = 1000.0;
    int fpsCap = 0;  // 0 = uncapped
    bool vsync = false;
    bool demo = false;
    bool fullscreen = false;
    int frames = 0;  // quit after N frames (benchmarks, screenshots)
    std::string screenshot;
};

void usage() {
    std::printf(
        "usage: slide [options]\n"
        "  --demo              start with the autopilot driving\n"
        "  --hz N              physics rate (default 1000)\n"
        "  --vsync             sync to the display instead of running uncapped\n"
        "  --fps N             cap the frame rate\n"
        "  --size WxH          window size (default 1600x900)\n"
        "  --fullscreen\n"
        "  --frames N          exit after N frames and print timing\n"
        "  --screenshot FILE   save the last frame (with --frames)\n");
}

Args parseArgs(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto next = [&]() -> const char* {
            if (i + 1 >= argc) {
                usage();
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--demo") a.demo = true;
        else if (arg == "--hz") a.hz = std::max(100.0, std::atof(next()));
        else if (arg == "--vsync") a.vsync = true;
        else if (arg == "--fps") a.fpsCap = std::atoi(next());
        else if (arg == "--fullscreen") a.fullscreen = true;
        else if (arg == "--frames") a.frames = std::atoi(next());
        else if (arg == "--screenshot") a.screenshot = next();
        else if (arg == "--size") std::sscanf(next(), "%dx%d", &a.width, &a.height);
        else {
            usage();
            std::exit(arg == "-h" || arg == "--help" ? 0 : 2);
        }
    }
    return a;
}

void loadFont() {
    static const char* kCandidates[] = {
#ifdef __EMSCRIPTEN__
        "/fonts/DejaVuSansMono.ttf",  // bundled into the web build
#endif
        "/System/Library/Fonts/SFNSMono.ttf",
        "/System/Library/Fonts/Supplemental/Andale Mono.ttf",
        "/System/Library/Fonts/Monaco.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
        "C:/Windows/Fonts/consola.ttf",
    };
    int codepoints[96 + 2];
    int n = 0;
    for (int c = 32; c < 127; ++c) codepoints[n++] = c;
    codepoints[n++] = 0xB0;  // degree sign
    for (const char* path : kCandidates) {
        if (!FileExists(path)) continue;
        Font font = LoadFontEx(path, 64, codepoints, n);
        if (font.texture.id == 0 || font.glyphCount == 0) continue;
        GenTextureMipmaps(&font.texture);
        SetTextureFilter(font.texture, TEXTURE_FILTER_TRILINEAR);
        app::uiFont() = font;
        return;
    }
}

// Turns keyboard (or gamepad) state into smooth pedal inputs. In automatic
// mode, holding brake at a standstill selects reverse and swaps the pedals,
// the usual arcade convention.
class ManualDriver {
public:
    DriverInput read(Vehicle& v, double dt) {
        const bool pad = IsGamepadAvailable(0);
        const bool forward = IsKeyDown(KEY_W) || IsKeyDown(KEY_UP);
        const bool back = IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN);
        Drivetrain& d = v.drivetrain();
        const bool reverse = d.gear() < 0;

        if (d.mode() == Drivetrain::Mode::Automatic && std::abs(v.localVelocity().x) < 0.6) {
            const bool wantSwap = reverse ? (forward && !back) : (back && !forward);
            swapTimer_ = wantSwap ? swapTimer_ + dt : 0.0;
            if (swapTimer_ > 0.3) {
                d.requestGear(reverse ? 1 : -1, v.drivenWheelOmega());
                swapTimer_ = 0.0;
            }
        } else {
            swapTimer_ = 0.0;
        }

        const bool gas = reverse ? back : forward;
        const bool stop = reverse ? forward : back;
        throttle_ = approach(throttle_, gas ? 1.0 : 0.0, (gas ? 5.0 : 10.0) * dt);
        brake_ = approach(brake_, stop ? 1.0 : 0.0, (stop ? 6.0 : 12.0) * dt);

        DriverInput in;
        in.throttle = throttle_;
        in.brake = brake_;
        in.steer = (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT) ? 1.0 : 0.0) -
                   (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT) ? 1.0 : 0.0);
        in.handbrake = IsKeyDown(KEY_SPACE);
        in.clutch = IsKeyDown(KEY_LEFT_SHIFT) ? 1.0 : 0.0;

        if (pad) {
            const double stick = -GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_X);
            if (std::abs(stick) > 0.08) in.steer = stick;
            const double rt = 0.5 * (GetGamepadAxisMovement(0, GAMEPAD_AXIS_RIGHT_TRIGGER) + 1.0);
            const double lt = 0.5 * (GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_TRIGGER) + 1.0);
            in.throttle = std::max(in.throttle, reverse ? lt : rt);
            in.brake = std::max(in.brake, reverse ? rt : lt);
            in.handbrake = in.handbrake || IsGamepadButtonDown(0, GAMEPAD_BUTTON_RIGHT_FACE_DOWN);
        }
        return in;
    }

    void reset() { throttle_ = brake_ = swapTimer_ = 0.0; }

private:
    double throttle_ = 0.0;
    double brake_ = 0.0;
    double swapTimer_ = 0.0;
};

app::CarPose poseOf(const Vehicle& v) { return {v.position(), v.heading()}; }

app::CarPose interpolate(const app::CarPose& a, const app::CarPose& b, double t) {
    return {a.position + (b.position - a.position) * t, a.heading + wrapAngle(b.heading - a.heading) * t};
}

#ifndef __EMSCRIPTEN__
std::string timestampedName(const char* prefix, const char* ext) {
    char buf[64];
    const std::time_t now = std::time(nullptr);
    std::strftime(buf, sizeof buf, "%Y%m%d_%H%M%S", std::localtime(&now));
    return std::string(prefix) + buf + ext;
}
#endif

// Everything the visualizer keeps between frames. It lives on the heap because
// in the browser the main loop keeps running after main() has returned.
struct App {
    explicit App(const Args& a)
        : args(a), vehicle({}, &track), pilot(track), lapTimer(track), world(track) {
        hs.autopilot = args.demo;
        resetCar();
        previous = poseOf(vehicle);
        camera.zoom = static_cast<float>(zoomBase);
        cameraTarget = vehicle.position();
        cameraAngle = vehicle.heading();
        stats.physicsHz = args.hz;
    }

    void toast(std::string message) {
        hs.toast = std::move(message);
        hs.toastAge = 0.0;
    }

    void resetCar() {
        const double start = track.length() - 15.0;
        const Vec2 t = track.tangentAt(start);
        vehicle.reset(track.pointAt(start), std::atan2(t.y, t.x));
        lapTimer = LapTimer(track);
        effects.clear();
        hud.clearHistory();
        driver.reset();
    }

    void handleKeys(double frameDt) {
        Drivetrain& gearbox = vehicle.drivetrain();
        DriverAidParams& aids = vehicle.aids().params();
        const auto manualShift = [&](int delta) {
            if (gearbox.mode() == Drivetrain::Mode::Automatic) gearbox.setMode(Drivetrain::Mode::Manual);
            gearbox.requestShift(delta, vehicle.drivenWheelOmega());
        };
        if (IsKeyPressed(KEY_E) || IsGamepadButtonPressed(0, GAMEPAD_BUTTON_RIGHT_TRIGGER_1)) manualShift(+1);
        if (IsKeyPressed(KEY_Q) || IsGamepadButtonPressed(0, GAMEPAD_BUTTON_LEFT_TRIGGER_1)) manualShift(-1);
        if (IsKeyPressed(KEY_M)) {
            const bool toManual = gearbox.mode() == Drivetrain::Mode::Automatic;
            gearbox.setMode(toManual ? Drivetrain::Mode::Manual : Drivetrain::Mode::Automatic);
            toast(toManual ? "Manual gearbox: E / Q to shift" : "Automatic gearbox");
        }
        if (IsKeyPressed(KEY_ONE)) toast((aids.absEnabled = !aids.absEnabled) ? "ABS on" : "ABS off");
        if (IsKeyPressed(KEY_TWO)) toast((aids.tcsEnabled = !aids.tcsEnabled) ? "Traction control on" : "Traction control off");
        if (IsKeyPressed(KEY_THREE)) toast((aids.escEnabled = !aids.escEnabled) ? "Stability control on" : "Stability control off");
        if (IsKeyPressed(KEY_P)) {
            hs.autopilot = !hs.autopilot;
            driver.reset();
        }
        if (IsKeyPressed(KEY_V)) xray = !xray;
        if (IsKeyPressed(KEY_C)) chaseCam = !chaseCam;
        if (IsKeyPressed(KEY_TAB)) showHud = !showHud;
        if (IsKeyPressed(KEY_F1) || IsKeyPressed(KEY_H)) hs.help = !hs.help;
        if (IsKeyPressed(KEY_R)) {
            resetCar();
            previous = poseOf(vehicle);
            accumulator = 0.0;
        }
#ifdef __EMSCRIPTEN__
        if (IsKeyPressed(KEY_T)) toast("Telemetry recording needs the desktop build");
#else
        if (IsKeyPressed(KEY_T)) {
            if (telemetry.isOpen()) {
                telemetry.close();
                toast("Saved " + std::to_string(telemetry.rows()) + " rows to " + telemetry.path());
            } else if (telemetry.open(timestampedName("slide_telemetry_", ".csv"),
                                      std::max(1, static_cast<int>(args.hz / 100.0)))) {
                toast("Recording 100 Hz telemetry to " + telemetry.path());
            }
        }
        if (IsKeyPressed(KEY_F12)) TakeScreenshot(timestampedName("slide_", ".png").c_str());
#endif
        const float wheel = GetMouseWheelMove();
        if (wheel != 0.0f) zoomBase = std::clamp(zoomBase * std::pow(1.15, wheel), 2.0, 40.0);
        if (IsKeyDown(KEY_EQUAL) || IsKeyDown(KEY_KP_ADD)) zoomBase = std::min(40.0, zoomBase * (1.0 + 1.5 * frameDt));
        if (IsKeyDown(KEY_MINUS) || IsKeyDown(KEY_KP_SUBTRACT)) zoomBase = std::max(2.0, zoomBase / (1.0 + 1.5 * frameDt));
        vehicle.engine().setStarter(IsKeyDown(KEY_I));
    }

    // Runs one rendered frame. Returns false once a --frames run is finished.
    bool frame() {
        const auto frameStart = Clock::now();
        const double frameDt = std::chrono::duration<double>(frameStart - lastFrame).count();
        lastFrame = frameStart;
        handleKeys(frameDt);

        // Fixed-step physics. Inputs are sampled once per frame; the autopilot
        // re-plans every step.
        const double dt = 1.0 / args.hz;
        const DriverInput manual = driver.read(vehicle, frameDt);
        accumulator += std::min(frameDt, 0.1);  // avoid a spiral of death after a stall
        const auto physicsStart = Clock::now();
        long steps = 0;
        while (accumulator >= dt) {
            previous = poseOf(vehicle);
            vehicle.step(hs.autopilot ? pilot.drive(vehicle) : manual, dt);
            if (lapTimer.update(vehicle.position(), vehicle.time())) {
                const bool best = lapTimer.lastLap() <= lapTimer.bestLap();
                char buf[64];
                std::snprintf(buf, sizeof buf, "Lap %d: %.3f s%s", lapTimer.laps(), lapTimer.lastLap(),
                              best ? "  (best)" : "");
                toast(buf);
            }
            telemetry.record(vehicle);
            accumulator -= dt;
            ++steps;
        }
        const double physicsSeconds = std::chrono::duration<double>(Clock::now() - physicsStart).count();
        const app::CarPose pose = interpolate(previous, poseOf(vehicle), accumulator / dt);

        effects.update(vehicle, frameDt);
        hud.sample(vehicle, frameDt);
        hs.toastAge += frameDt;
        hs.recording = telemetry.isOpen();
        hs.laps = lapTimer.laps();
        hs.currentLap = lapTimer.currentLap(vehicle.time());
        hs.lastLap = lapTimer.lastLap();
        hs.bestLap = lapTimer.bestLap();

        // Camera: eased follow with look-ahead, zooming out with speed.
        const double follow = 1.0 - std::exp(-8.0 * frameDt);
        const Vec2 lookAhead = pose.position + vehicle.velocity() * 0.35;
        cameraTarget += (lookAhead - cameraTarget) * follow;
        cameraAngle += wrapAngle(pose.heading - cameraAngle) * (1.0 - std::exp(-4.0 * frameDt));
        const float sw = static_cast<float>(GetScreenWidth()), sh = static_cast<float>(GetScreenHeight());
        camera.target = app::rl(cameraTarget);
        camera.offset = {0.5f * sw, chaseCam ? 0.58f * sh : 0.5f * sh};
        camera.rotation = chaseCam ? static_cast<float>(toDeg(cameraAngle) - 90.0) : 0.0f;
        const double targetZoom = zoomBase * (sh / 900.0) / (1.0 + vehicle.speed() / 80.0);
        camera.zoom += static_cast<float>((targetZoom - camera.zoom) * (1.0 - std::exp(-3.0 * frameDt)));

        BeginDrawing();
        BeginMode2D(camera);
        world.drawGround(camera);
        world.drawTrack();
        effects.drawMarks();
        world.drawCar(vehicle, pose, xray);
        if (xray) world.drawVectors(vehicle, pose);
        effects.drawSmoke();
        EndMode2D();
        if (showHud) hud.draw(vehicle, stats, hs);
        const bool finalFrame = args.frames > 0 && frame_ + 1 >= args.frames;
        if (finalFrame && !args.screenshot.empty()) {
            // Flush the batched HUD and read the back buffer before it is
            // presented; TakeScreenshot() would drop the directory part.
            rlDrawRenderBatchActive();
            Image shot = LoadImageFromScreen();
            ExportImage(shot, args.screenshot.c_str());
            UnloadImage(shot);
        }
        EndDrawing();

        // Timing statistics, refreshed twice a second.
        ++windowFrames;
        windowSteps += steps;
        windowTime += frameDt;
        windowPhysics += physicsSeconds;
        if (windowTime >= 0.5) {
            stats.fps = windowFrames / windowTime;
            stats.frameMs = 1000.0 * windowTime / windowFrames;
            stats.stepMicros = windowSteps > 0 ? 1e6 * windowPhysics / windowSteps : 0.0;
            stats.stepsPerFrame = static_cast<double>(windowSteps) / windowFrames;
            windowTime = windowPhysics = 0.0;
            windowFrames = windowSteps = 0;
        }
        if (++frame_ > 60) {  // skip warm-up frames
            ++totalFrames;
            totalSteps += steps;
            totalTime += frameDt;
            totalPhysics += physicsSeconds;
        }
        return !finalFrame;
    }

    void printSummary() const {
        if (totalFrames == 0) return;
        std::printf("frames %ld  avg %.0f fps (%.3f ms/frame)  physics %.2f us/step at %.0f Hz, "
                    "%.2f%% of frame time  sim time %.1f s\n",
                    totalFrames, totalFrames / totalTime, 1000.0 * totalTime / totalFrames,
                    totalSteps > 0 ? 1e6 * totalPhysics / totalSteps : 0.0, args.hz,
                    100.0 * totalPhysics / totalTime, vehicle.time());
    }

    Args args;
    const Track track = Track::circuit();
    Vehicle vehicle;
    Autopilot pilot;
    LapTimer lapTimer;
    app::WorldRenderer world;
    app::Effects effects;
    app::Hud hud;
    TelemetryLog telemetry;
    ManualDriver driver;

    app::HudState hs;
    bool xray = true;
    bool showHud = true;
    bool chaseCam = true;
    double zoomBase = 12.0;  // pixels per meter at rest

    double accumulator = 0.0;
    app::CarPose previous;
    Camera2D camera{};
    Vec2 cameraTarget;
    double cameraAngle = 0.0;

    app::FrameStats stats;
    double windowTime = 0.0, windowPhysics = 0.0;
    long windowFrames = 0, windowSteps = 0;
    double totalTime = 0.0, totalPhysics = 0.0;
    long totalFrames = 0, totalSteps = 0;
    long frame_ = 0;
    Clock::time_point lastFrame = Clock::now();
};

}  // namespace

int main(int argc, char** argv) {
    Args args = parseArgs(argc, argv);

    unsigned flags = FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE;
#ifdef __EMSCRIPTEN__
    // Fill the browser window; the browser paces frames to the display.
    args.width = EM_ASM_INT(return window.innerWidth);
    args.height = EM_ASM_INT(return window.innerHeight);
#else
    flags |= FLAG_WINDOW_HIGHDPI;
    if (args.vsync) flags |= FLAG_VSYNC_HINT;
#endif
    SetConfigFlags(flags);
    SetTraceLogLevel(LOG_WARNING);
    InitWindow(args.width, args.height, "Slide - vehicle dynamics");
    if (args.fullscreen) ToggleBorderlessWindowed();
    SetTargetFPS(args.fpsCap);
    loadFont();

    auto* app = new App(args);
#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop_arg([](void* p) { static_cast<App*>(p)->frame(); }, app, 0, true);
#else
    while (!WindowShouldClose() && app->frame()) {
    }
    app->printSummary();
    delete app;
    CloseWindow();
#endif
    return 0;
}
