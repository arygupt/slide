#pragma once

// Vehicle parameter sets. Every quantity is SI; defaults describe a ~1350 kg
// rear-drive sports coupe with a 3.2 L naturally aspirated six.

#include <array>

#include "slide/Math.h"

namespace slide {

struct ChassisParams {
    double mass = 1350.0;               // total, including unsprung [kg]
    double yawInertia = 2100.0;         // Izz [kg m^2]
    double pitchInertia = 1700.0;       // sprung mass Iyy [kg m^2]
    double rollInertia = 450.0;         // sprung mass Ixx [kg m^2]
    double wheelbase = 2.60;            // [m]
    double frontWeightFraction = 0.52;  // static
    double cgHeight = 0.48;             // [m]
    double length = 4.35;               // body, for rendering [m]
    double width = 1.82;

    double airDensity = 1.225;     // [kg/m^3]
    double dragArea = 0.62;        // Cd * A [m^2]
    double downforceArea = 0.25;   // Cl * A [m^2]
    double aeroFrontBalance = 0.45;

    // Distance from the CG to each axle.
    double cgToFront() const { return wheelbase * (1.0 - frontWeightFraction); }
    double cgToRear() const { return wheelbase * frontWeightFraction; }
};

struct AxleParams {
    double track = 1.56;            // [m]
    double unsprungMass = 40.0;     // per corner [kg]
    double wheelInertia = 1.3;      // wheel + tire + rotor (+ halfshaft) [kg m^2]
    double springRate = 35000.0;    // wheel rate [N/m]
    double bumpDamping = 2000.0;    // [N s/m]
    double reboundDamping = 3300.0; // [N s/m]
    double antiRollRate = 28000.0;  // [N/m of left-right travel difference]
    double rollCenterHeight = 0.05; // [m]
    double bumpTravel = 0.075;      // compression before the bump stop [m]
    double bumpStopRate = 3.0e6;    // progressive: F = k * excess^2 [N/m^2]
    double maxBrakeTorque = 2400.0; // per wheel at full pedal [N m]
};

struct TireParams {
    double radius = 0.32;           // [m]
    double width = 0.245;           // rendering only [m]
    double nominalLoad = 4000.0;    // Fz0 [N]
    double mu = 1.10;               // peak friction coefficient at Fz0
    double loadSensitivity = 0.12;  // fractional mu lost per +100% load

    // Magic Formula pure-slip shape factors; the peak D comes from mu * Fz.
    double Bx = 13.0, Cx = 1.50, Ex = 0.0;   // longitudinal, input = slip ratio
    double By = 11.5, Cy = 1.45, Ey = -0.3;  // lateral, input = slip angle [rad]

    double relaxationLengthX = 0.12;  // [m]
    double relaxationLengthY = 0.30;  // [m]
    double maxSlipRatio = 1.5;        // contact-patch deflection limits
    double maxSlipAngle = deg(60.0);

    // Extra contact damping that fades out above lowSpeedThreshold, so a
    // parked car settles instead of ringing on its tire stiffness.
    double lowSpeedDampingX = 2500.0;  // [N per m/s of slip]
    double lowSpeedDampingY = 6000.0;
    double lowSpeedThreshold = 3.0;    // [m/s]
};

struct SteeringParams {
    double maxAngle = deg(34.0);       // road wheel angle at full lock
    double ackermann = 0.85;           // 1 = full Ackermann, 0 = parallel
    double inputRate = 3.0;            // normalized steering travel per second
    double speedReference = 30.0;      // lock reduces by half at this speed [m/s]
    double minSpeedFactor = 0.30;
};

struct EngineParams {
    double displacement = 3.2e-3;      // [m^3]
    double inertia = 0.17;             // crank + flywheel + clutch [kg m^2]
    double idleRpm = 850.0;
    double stallRpm = 380.0;
    double redlineRpm = 7400.0;
    double limiterRpm = 7600.0;
    double limiterHysteresis = 250.0;
    double starterTorque = 90.0;       // [N m]

    // Combustion: indicated torque from trapped air mass.
    double peakIndicatedEfficiency = 0.38;
    double partLoadEfficiency = 0.40;  // fraction of peak at zero manifold pressure
    double fuelLhv = 44.0e6;           // [J/kg]
    double stoichAfr = 14.7;
    double wotLambda = 0.88;           // enrichment at full load (fuel flow only)

    // Volumetric efficiency curve.
    double vePeak = 0.98;
    double vePeakRpm = 5200.0;
    double veLowDrop = 0.22;           // VE lost toward 0 rpm
    double veHighDrop = 0.16;          // VE lost at the limiter

    // Intake: electronic throttle, idle air valve and manifold plenum.
    double throttleDiameter = 0.074;   // [m]
    double throttleDischarge = 0.82;   // Cd
    double throttleClosedAngle = deg(7.0);
    double throttleTimeConstant = 0.04;  // drive-by-wire actuator [s]
    double idleValveMaxArea = 6.0e-5;  // [m^2]
    double leakArea = 1.5e-5;          // closed-plate leakage and stop [m^2]
    double overrunAirArea = 4.0e-5;    // decel dashpot opening during fuel cut [m^2]
    double manifoldVolume = 4.5e-3;    // [m^3]
    double ambientPressure = 101325.0; // [Pa]
    double ambientTemperature = 298.15;// [K]
    double exhaustPressureRatio = 1.06;

    // Friction mean effective pressure: a + b N + c N^2 with N in krpm [Pa].
    double fmepA = 0.40e5;
    double fmepB = 0.08e5;
    double fmepC = 0.018e5;

    // Idle speed PI controller acting on the idle air valve.
    double idleKp = 4.0e-7;            // [m^2 per rpm]
    double idleKi = 1.2e-6;            // [m^2 per rpm s]
};

struct DrivetrainParams {
    static constexpr int kGearCount = 6;
    std::array<double, kGearCount> gearRatios{3.62, 2.36, 1.69, 1.31, 1.00, 0.82};
    double reverseRatio = 3.45;
    double finalDrive = 3.62;
    double efficiency = 0.92;

    double clutchMaxTorque = 520.0;    // [N m]
    double clutchEngageRate = 6.0;     // engagement per second
    double clutchReleaseRate = 18.0;
    double shiftTime = 0.16;           // [s]
    double overRevGuardRpm = 7800.0;   // refuse downshifts that would exceed this

    // Automatic clutch: engagement follows engine speed until it locks up.
    double autoClutchStartRpm = 1200.0;
    double autoClutchFullRpm = 3000.0;
    double autoClutchReleaseRpm = 950.0;

    // Automatic shift schedule, interpolated by throttle position.
    double upshiftRpmLight = 2700.0;
    double upshiftRpmFull = 7200.0;
    double downshiftRpmLight = 1400.0;
    double downshiftRpmFull = 4000.0;

    // Clutch-pack limited slip differential.
    double lsdViscous = 350.0;         // [N m per rad/s]
    double lsdPreload = 60.0;          // [N m]
    double lsdPowerLock = 0.30;        // extra locking per N m of input
};

struct BrakeParams {
    double hydraulicTimeConstant = 0.035;  // [s]
    double handbrakeTorque = 2200.0;       // rear wheels [N m]
};

struct DriverAidParams {
    bool absEnabled = true;
    bool tcsEnabled = true;
    bool escEnabled = true;

    double absSlipTarget = 0.10;
    double absKp = 6.0;                // pressure fraction per unit slip
    double absKi = 40.0;               // pressure fraction per second per unit slip
    double absReleaseRate = 14.0;      // pressure fraction per second
    double absApplyRate = 5.0;
    double absMinSpeed = 2.0;          // [m/s]

    double tcsSlipTarget = 0.12;
    double tcsSlipSpeed = 1.5;         // slip threshold near standstill [m/s]
    double tcsKp = 4.0;
    double tcsKi = 12.0;
    double tcsMaxCut = 0.95;

    double escUndersteerGradient = deg(1.5) / kGravity;  // [rad per m/s^2]
    double escDeadband = 0.06;         // [rad/s]
    double escGain = 2500.0;           // [N m per rad/s]
    double escMaxTorque = 1800.0;      // [N m]
    double escMinSpeed = 5.0;          // [m/s]
    double escSideslipLimit = deg(10.0);
};

struct VehicleParams {
    ChassisParams chassis;
    AxleParams front;
    AxleParams rear{.track = 1.58,
                    .unsprungMass = 42.0,
                    .wheelInertia = 1.5,
                    .springRate = 40000.0,
                    .bumpDamping = 2000.0,
                    .reboundDamping = 3400.0,
                    .antiRollRate = 12000.0,
                    .rollCenterHeight = 0.10,
                    .maxBrakeTorque = 1300.0};
    TireParams frontTire;  // 245/40 R18
    TireParams rearTire{.width = 0.275, .mu = 1.12, .By = 14.5};  // staggered 275s
    SteeringParams steering;
    EngineParams engine;
    DrivetrainParams drivetrain;
    BrakeParams brakes;
    DriverAidParams aids;

    double unsprungMass() const { return 2.0 * (front.unsprungMass + rear.unsprungMass); }
    double sprungMass() const { return chassis.mass - unsprungMass(); }
};

}  // namespace slide
