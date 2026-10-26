#include "slide/Telemetry.h"

#include "slide/Vehicle.h"

namespace slide {
namespace {

// Calls `emit(name, value)` for every channel in a fixed order. The same
// visitor produces the CSV header and each row, so the two cannot drift.
template <typename Emit>
void visitChannels(const Vehicle& v, Emit&& emit) {
    const Vec2 local = v.localVelocity();
    const Vec2 accel = v.localAcceleration();
    const Engine::Readout& e = v.engine().readout();
    const Drivetrain& d = v.drivetrain();
    const DriverInput& in = v.lastInput();
    const Suspension::BodyState& body = v.suspension().body();

    emit("time_s", v.time());
    emit("x_m", v.position().x);
    emit("y_m", v.position().y);
    emit("heading_deg", toDeg(v.heading()));
    emit("speed_kmh", v.speed() * 3.6);
    emit("vx_ms", local.x);
    emit("vy_ms", local.y);
    emit("yaw_rate_dps", toDeg(v.yawRate()));
    emit("sideslip_deg", toDeg(v.sideslip()));
    emit("ax_g", accel.x / kGravity);
    emit("ay_g", accel.y / kGravity);
    emit("throttle", in.throttle);
    emit("brake", in.brake);
    emit("steer", in.steer);
    emit("steer_angle_deg", toDeg(v.steerAngle()));
    emit("gear", d.gear());
    emit("rpm", v.engine().rpm());
    emit("engine_torque_nm", e.netTorque);
    emit("engine_power_kw", e.netTorque * v.engine().omega() / 1000.0);
    emit("map_kpa", e.manifoldPressure / 1000.0);
    emit("throttle_angle_deg", toDeg(e.throttleAngle));
    emit("airflow_gs", e.cylinderAirflow * 1000.0);
    emit("fuel_flow_gs", e.fuelFlow * 1000.0);
    emit("clutch_engagement", d.clutchEngagement());
    emit("clutch_torque_nm", d.clutchTorque());
    emit("clutch_slip_rpm", d.clutchSlip() * kRadToRpm);
    emit("lsd_torque_nm", d.lsdTorque());
    emit("tcs_cut", v.aids().torqueCut());
    emit("abs_active", v.aids().absActive() ? 1.0 : 0.0);
    emit("esc_active", v.aids().escActive() ? 1.0 : 0.0);
    emit("heave_mm", body.heave * 1000.0);
    emit("pitch_deg", toDeg(body.pitch));
    emit("roll_deg", toDeg(body.roll));
    emit("drag_n", v.dragForce());
    emit("downforce_n", v.downforce());

    static constexpr const char* kNames[kCornerCount][7] = {
        {"fl_load_n", "fl_slip_ratio", "fl_slip_angle_deg", "fl_fx_n", "fl_fy_n", "fl_travel_mm", "fl_wheel_rpm"},
        {"fr_load_n", "fr_slip_ratio", "fr_slip_angle_deg", "fr_fx_n", "fr_fy_n", "fr_travel_mm", "fr_wheel_rpm"},
        {"rl_load_n", "rl_slip_ratio", "rl_slip_angle_deg", "rl_fx_n", "rl_fy_n", "rl_travel_mm", "rl_wheel_rpm"},
        {"rr_load_n", "rr_slip_ratio", "rr_slip_angle_deg", "rr_fx_n", "rr_fy_n", "rr_travel_mm", "rr_wheel_rpm"},
    };
    for (int i = 0; i < kCornerCount; ++i) {
        const Wheel& w = v.wheel(i);
        const Suspension::CornerState& c = v.suspension().corner(i);
        const auto& n = kNames[i];
        emit(n[0], c.tireLoad);
        emit(n[1], w.tire.slipRatio);
        emit(n[2], toDeg(w.tire.slipAngle));
        emit(n[3], w.tire.Fx);
        emit(n[4], w.tire.Fy);
        emit(n[5], c.compression * 1000.0);
        emit(n[6], w.omega * kRadToRpm);
    }
}

}  // namespace

bool TelemetryLog::open(const std::string& path, int decimation) {
    close();
    file_ = std::fopen(path.c_str(), "w");
    path_ = path;
    decimation_ = decimation < 1 ? 1 : decimation;
    calls_ = rows_ = 0;
    return file_ != nullptr;
}

void TelemetryLog::record(const Vehicle& vehicle) {
    if (!file_ || calls_++ % decimation_ != 0) return;
    if (rows_ == 0) {
        bool first = true;
        visitChannels(vehicle, [&](const char* name, double) {
            std::fprintf(file_, first ? "%s" : ",%s", name);
            first = false;
        });
        std::fputc('\n', file_);
    }
    values_.clear();
    visitChannels(vehicle, [&](const char*, double value) { values_.push_back(value); });
    for (size_t i = 0; i < values_.size(); ++i) {
        std::fprintf(file_, i == 0 ? "%.6g" : ",%.6g", values_[i]);
    }
    std::fputc('\n', file_);
    ++rows_;
}

void TelemetryLog::close() {
    if (file_) std::fclose(file_);
    file_ = nullptr;
}

}  // namespace slide
