// slide-bench: headless test stand for the vehicle model.
//
// Runs the same procedures a powertrain or vehicle dynamics test cell would:
// a speed-controlled engine dyno sweep, straight-line launch and braking,
// constant-radius skidpad, an ISO 7401-style step steer, autopilot laps, and a
// raw throughput benchmark of the physics step.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "slide/Autopilot.h"
#include "slide/Telemetry.h"
#include "slide/Track.h"
#include "slide/Vehicle.h"

using namespace slide;

namespace {

struct Options {
    std::string scenario = "all";
    double hz = 1000.0;
    std::string csv;
    double seconds = 120.0;
    int laps = 3;
    bool abs = true;
    bool tcs = true;
    bool esc = true;
};

double dtOf(const Options& o) { return 1.0 / o.hz; }

VehicleParams paramsFor(const Options& o) {
    VehicleParams p;
    p.aids.absEnabled = o.abs;
    p.aids.tcsEnabled = o.tcs;
    p.aids.escEnabled = o.esc;
    return p;
}

void heading(const char* title) {
    std::printf("\n== %s ", title);
    for (size_t i = std::char_traits<char>::length(title); i < 66; ++i) std::putchar('=');
    std::putchar('\n');
}

// -- Engine dyno -------------------------------------------------------------

struct DynoPoint {
    double rpm, torque, power, map, airflow, ve, bsfc;
};

// Speed-controlled absorber: the dyno pins crank speed and the engine settles
// at wide-open throttle; brake torque is read once the manifold has filled.
std::vector<DynoPoint> runDyno(const Options& o, double pedal) {
    const double dt = dtOf(o);
    std::vector<DynoPoint> points;
    for (double rpm = 1000.0; rpm <= 7600.0 + 1e-9; rpm += 250.0) {
        Engine engine(EngineParams{});
        double torque = 0.0, fuel = 0.0, map = 0.0, air = 0.0;
        int samples = 0;
        for (double t = 0.0; t < 1.5; t += dt) {
            engine.forceSpeed(rpm * kRpmToRad);
            engine.update(pedal, 0.0, dt);
            if (t > 1.2) {
                const Engine::Readout& r = engine.readout();
                torque += r.netTorque;
                fuel += r.fuelFlow;
                map += r.manifoldPressure;
                air += r.cylinderAirflow;
                ++samples;
            }
        }
        torque /= samples;
        const double power = torque * rpm * kRpmToRad;
        const double bsfc = power > 1000.0 ? (fuel / samples) * 3.6e9 / power : 0.0;  // g/kWh
        points.push_back({rpm, torque, power / 1000.0, map / samples / 1000.0,
                          air / samples * 1000.0, engine.volumetricEfficiency(rpm), bsfc});
    }
    return points;
}

void dyno(const Options& o) {
    heading("Engine dyno: WOT sweep, 3.2 L NA I6");
    const auto points = runDyno(o, 1.0);
    std::printf("   rpm  torque Nm   power kW   MAP kPa   air g/s     VE   BSFC g/kWh\n");
    const DynoPoint* peakTorque = &points.front();
    const DynoPoint* peakPower = &points.front();
    for (const DynoPoint& p : points) {
        std::printf("%6.0f %10.1f %10.1f %9.1f %9.1f %6.3f %12.0f\n", p.rpm, p.torque, p.power,
                    p.map, p.airflow, p.ve, p.bsfc);
        if (p.torque > peakTorque->torque) peakTorque = &p;
        if (p.power > peakPower->power) peakPower = &p;
    }
    std::printf("peak torque %.0f Nm @ %.0f rpm, peak power %.0f kW (%.0f hp) @ %.0f rpm\n",
                peakTorque->torque, peakTorque->rpm, peakPower->power, peakPower->power * 1.341,
                peakPower->rpm);

    if (!o.csv.empty()) {
        if (std::FILE* f = std::fopen(o.csv.c_str(), "w")) {
            std::fprintf(f, "rpm,torque_nm,power_kw,map_kpa,airflow_gs,ve,bsfc_g_kwh\n");
            for (const DynoPoint& p : points) {
                std::fprintf(f, "%.0f,%.3f,%.3f,%.3f,%.3f,%.4f,%.1f\n", p.rpm, p.torque, p.power,
                             p.map, p.airflow, p.ve, p.bsfc);
            }
            std::fclose(f);
            std::printf("wrote %s\n", o.csv.c_str());
        }
    }
}

// -- Straight-line tests -----------------------------------------------------

void launch(const Options& o) {
    heading("Launch: standing start, full throttle, automatic shifts");
    const double dt = dtOf(o);
    Vehicle v(paramsFor(o));
    TelemetryLog log;
    if (!o.csv.empty()) log.open(o.csv, std::max(1, static_cast<int>(o.hz / 100.0)));

    struct Mark { const char* label; double speed; double time = -1.0; };
    std::vector<Mark> marks{{"0-60 mph", 26.8224}, {"0-100 km/h", 100 / 3.6},
                            {"0-160 km/h", 160 / 3.6}, {"0-200 km/h", 200 / 3.6}};
    double quarterTime = -1.0, quarterSpeed = 0.0, peakG = 0.0;
    DriverInput in;
    in.throttle = 1.0;
    while (v.time() < 40.0) {
        v.step(in, dt);
        log.record(v);
        peakG = std::max(peakG, v.localAcceleration().x / kGravity);
        for (Mark& m : marks) {
            if (m.time < 0.0 && v.speed() >= m.speed) m.time = v.time();
        }
        if (quarterTime < 0.0 && v.position().x >= 402.336) {
            quarterTime = v.time();
            quarterSpeed = v.speed();
        }
    }
    for (const Mark& m : marks) std::printf("%-12s %6.2f s\n", m.label, m.time);
    std::printf("1/4 mile     %6.2f s @ %.1f km/h\n", quarterTime, quarterSpeed * 3.6);
    std::printf("peak accel   %6.2f g, speed after 40 s: %.0f km/h in gear %d\n", peakG,
                v.speed() * 3.6, v.drivetrain().gear());
    if (log.isOpen()) std::printf("wrote %ld rows to %s\n", log.rows(), log.path().c_str());
}

double stoppingDistance(const Options& o, bool abs, double fromKmh, double* meanDecel) {
    VehicleParams p = paramsFor(o);
    p.aids.absEnabled = abs;
    Vehicle v(p);
    v.reset({}, 0.0, fromKmh / 3.6);
    DriverInput in;
    in.brake = 1.0;
    const double dt = dtOf(o);
    while (v.speed() > 0.05 && v.time() < 20.0) v.step(in, dt);
    if (meanDecel) *meanDecel = fromKmh / 3.6 / v.time() / kGravity;
    return v.position().length();
}

void brake(const Options& o) {
    heading("Braking: full pedal from speed, ABS on vs locked wheels");
    std::printf("  from        ABS on              ABS off\n");
    for (double kmh : {100.0, 200.0}) {
        double gOn = 0.0, gOff = 0.0;
        const double on = stoppingDistance(o, true, kmh, &gOn);
        const double off = stoppingDistance(o, false, kmh, &gOff);
        std::printf("%4.0f km/h  %6.1f m (%.2f g)    %6.1f m (%.2f g)   ABS saves %.0f%%\n", kmh, on,
                    gOn, off, gOff, 100.0 * (off - on) / off);
    }
}

// -- Handling tests ----------------------------------------------------------

// Pure pursuit around a circle centered at the origin, counter-clockwise.
DriverInput followCircle(const Vehicle& v, double radius, double speed) {
    const Vec2 p = v.position();
    const double angle = std::atan2(p.y, p.x);
    const double lookahead = std::clamp(0.5 * v.speed(), 4.0, 15.0);
    const Vec2 target = Vec2{std::cos(angle + lookahead / radius), std::sin(angle + lookahead / radius)} * radius;
    const Vec2 rear = v.worldPoint({-v.params().chassis.cgToRear(), 0.0});
    const Vec2 local = Rot2(v.heading()).applyInverse(target - rear);
    const double delta = std::atan(2.0 * v.params().chassis.wheelbase * std::sin(std::atan2(local.y, local.x)) /
                                   std::max(local.length(), 1.0));
    DriverInput in;
    in.steer = std::clamp(delta / v.maxSteerAngle(), -1.0, 1.0);
    const double error = speed - v.localVelocity().x;
    in.throttle = std::clamp(0.2 + 0.5 * error, 0.0, 1.0);
    in.brake = error < -1.0 ? std::clamp(-0.1 * error, 0.0, 1.0) : 0.0;
    return in;
}

void skidpad(const Options& o) {
    heading("Skidpad: 40 m radius, speed ramp, stability control off");
    constexpr double kRadius = 40.0;
    Options local = o;
    local.esc = false;
    Vehicle v(paramsFor(local));
    v.reset({kRadius, 0.0}, kPi / 2.0, 8.0);
    const double dt = dtOf(o);

    // Linear regression of steer angle on lateral g gives the understeer
    // gradient: delta = L/R + K * ay.
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    int n = 0;
    double maxLateral = 0.0;
    double maxSpeed = 0.0;
    while (v.time() < 120.0) {
        const double target = 8.0 + 0.15 * v.time();
        v.step(followCircle(v, kRadius, target), dt);
        const double radiusError = v.position().length() - kRadius;
        const double ay = v.localAcceleration().y / kGravity;
        if (v.time() > 5.0 && std::abs(radiusError) < 1.0 && std::abs(v.sideslip()) < deg(6.0)) {
            if (ay > maxLateral) {
                maxLateral = ay;
                maxSpeed = v.speed();
            }
            if (ay > 0.1 && ay < 0.4) {
                const double steer = toDeg(v.steerAngle());
                sx += ay; sy += steer; sxx += ay * ay; sxy += ay * steer;
                ++n;
            }
        }
        if (std::abs(radiusError) > 6.0) break;  // left the circle
    }
    const double gradient = n > 2 ? (n * sxy - sx * sy) / (n * sxx - sx * sx) : 0.0;
    std::printf("max steady lateral   %.2f g at %.1f km/h\n", maxLateral, maxSpeed * 3.6);
    std::printf("understeer gradient  %.2f deg/g (%s)\n", gradient,
                gradient > 0.0 ? "understeer, stable" : "oversteer");
}

void stepSteer(const Options& o) {
    heading("Step steer: 80 km/h, ISO 7401 style yaw response");
    Vehicle v(paramsFor(o));
    v.reset({}, 0.0, 80.0 / 3.6);
    const double dt = dtOf(o);
    const double stepTime = 1.0;
    std::vector<std::pair<double, double>> yaw;
    while (v.time() < stepTime + 4.0) {
        DriverInput in;
        in.steer = v.time() >= stepTime ? 0.08 : 0.0;
        in.throttle = std::clamp(0.3 + 0.5 * (80.0 / 3.6 - v.localVelocity().x), 0.0, 1.0);
        v.step(in, dt);
        if (v.time() >= stepTime) yaw.emplace_back(v.time() - stepTime, v.yawRate());
    }
    const double steady = yaw.back().second;
    double peak = 0.0, t50 = -1, t90 = -1, tPeak = 0.0;
    for (auto [t, r] : yaw) {
        if (t50 < 0 && r >= 0.5 * steady) t50 = t;
        if (t90 < 0 && r >= 0.9 * steady) t90 = t;
        if (r > peak) { peak = r; tPeak = t; }
    }
    std::printf("steady yaw rate      %.1f deg/s, lateral %.2f g\n", toDeg(steady),
                v.localAcceleration().y / kGravity);
    std::printf("response time 50%%   %.0f ms, 90%%: %.0f ms (input ramp included)\n", t50 * 1000.0,
                t90 * 1000.0);
    std::printf("peak overshoot       %.1f%% at %.0f ms\n", 100.0 * (peak - steady) / steady,
                tPeak * 1000.0);
}

void laps(const Options& o) {
    heading("Circuit: autopilot laps");
    const Track track = Track::circuit();
    Vehicle v(paramsFor(o), &track);
    const double start = track.length() - 20.0;
    v.reset(track.pointAt(start), std::atan2(track.tangentAt(start).y, track.tangentAt(start).x));
    Autopilot pilot(track);
    LapTimer timer(track);
    TelemetryLog log;
    if (!o.csv.empty()) log.open(o.csv, std::max(1, static_cast<int>(o.hz / 100.0)));

    const double dt = dtOf(o);
    double maxSpeed = 0.0, maxLateral = 0.0, offTrack = 0.0;
    while (timer.laps() < o.laps && v.time() < 600.0) {
        v.step(pilot.drive(v), dt);
        log.record(v);
        maxSpeed = std::max(maxSpeed, v.speed());
        maxLateral = std::max(maxLateral, std::abs(v.localAcceleration().y) / kGravity);
        if (std::abs(track.project(v.position()).lateral) > track.halfWidth()) offTrack += dt;
        if (timer.update(v.position(), v.time())) {
            std::printf("lap %d  %7.3f s\n", timer.laps(), timer.lastLap());
        }
    }
    std::printf("track %.0f m, best %.3f s (avg %.1f km/h), top %.0f km/h, peak %.2f g lateral, "
                "%.1f s off track\n",
                track.length(), timer.bestLap(), track.length() / timer.bestLap() * 3.6,
                maxSpeed * 3.6, maxLateral, offTrack);
    if (log.isOpen()) std::printf("wrote %ld rows to %s\n", log.rows(), log.path().c_str());
}

// -- Throughput --------------------------------------------------------------

void bench(const Options& o) {
    heading("Benchmark: physics step throughput (circuit + autopilot)");
    const Track track = Track::circuit();
    Vehicle v(paramsFor(o), &track);
    const double start = track.length() - 20.0;
    v.reset(track.pointAt(start), std::atan2(track.tangentAt(start).y, track.tangentAt(start).x));
    Autopilot pilot(track);
    const double dt = dtOf(o);
    const long steps = static_cast<long>(o.seconds * o.hz);

    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    double checksum = 0.0;
    for (long i = 0; i < steps; ++i) {
        v.step(pilot.drive(v), dt);
        checksum += v.speed();
    }
    const double wall = std::chrono::duration<double>(Clock::now() - t0).count();
    const double perStep = wall / static_cast<double>(steps);
    std::printf("simulated %.0f s at %.0f Hz = %ld steps in %.3f s wall\n", o.seconds, o.hz, steps, wall);
    std::printf("%.2f us/step, %.0f steps/s, %.0fx faster than real time\n", perStep * 1e6,
                1.0 / perStep, o.seconds / wall);
    std::printf("per 300 FPS frame (%.2f steps): %.1f us of physics, %.2f%% of the frame budget\n",
                o.hz / 300.0, perStep * o.hz / 300.0 * 1e6, perStep * o.hz / 300.0 * 300.0 * 100.0);
    std::printf("(checksum %.3f)\n", checksum / static_cast<double>(steps));
}

void usage() {
    std::printf(
        "usage: slide-bench [scenario] [options]\n\n"
        "scenarios:\n"
        "  all         run everything below (default)\n"
        "  dyno        WOT engine dyno sweep (torque, power, MAP, airflow, BSFC)\n"
        "  launch      0-100 km/h, quarter mile\n"
        "  brake       stopping distance with and without ABS\n"
        "  skidpad     max lateral g and understeer gradient\n"
        "  step-steer  yaw rate response to a steering step\n"
        "  laps        autopilot laps of the circuit\n"
        "  bench       physics throughput\n\n"
        "options:\n"
        "  --hz N        physics rate (default 1000)\n"
        "  --csv FILE    write telemetry (launch, laps) or the dyno table\n"
        "  --laps N      laps for the laps scenario (default 3)\n"
        "  --seconds N   simulated time for bench (default 120)\n"
        "  --no-abs --no-tcs --no-esc   disable driver aids\n");
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto value = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %s\n", arg.c_str());
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--hz") o.hz = std::atof(value().c_str());
        else if (arg == "--csv") o.csv = value();
        else if (arg == "--laps") o.laps = std::atoi(value().c_str());
        else if (arg == "--seconds") o.seconds = std::atof(value().c_str());
        else if (arg == "--no-abs") o.abs = false;
        else if (arg == "--no-tcs") o.tcs = false;
        else if (arg == "--no-esc") o.esc = false;
        else if (arg == "-h" || arg == "--help") { usage(); return 0; }
        else if (!arg.empty() && arg[0] != '-') o.scenario = arg;
        else { usage(); return 2; }
    }
    if (o.hz < 100.0) {
        std::fprintf(stderr, "--hz must be at least 100\n");
        return 2;
    }

    const std::map<std::string, std::function<void(const Options&)>> scenarios{
        {"dyno", dyno},       {"launch", launch}, {"brake", brake}, {"skidpad", skidpad},
        {"step-steer", stepSteer}, {"laps", laps}, {"bench", bench}};
    if (o.scenario == "all") {
        if (!o.csv.empty()) std::printf("note: --csv is ignored for 'all'\n");
        Options quiet = o;
        quiet.csv.clear();
        for (const char* name : {"dyno", "launch", "brake", "skidpad", "step-steer", "laps", "bench"}) {
            scenarios.at(name)(quiet);
        }
        return 0;
    }
    const auto it = scenarios.find(o.scenario);
    if (it == scenarios.end()) {
        std::fprintf(stderr, "unknown scenario '%s'\n\n", o.scenario.c_str());
        usage();
        return 2;
    }
    it->second(o);
    return 0;
}
