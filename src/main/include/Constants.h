#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Translation2d.h>
#include <units/length.h>

#include <string>

namespace FieldConstants {
inline const frc::AprilTagFieldLayout layout =
    frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltWelded);

inline const units::meter_t fieldLength = layout.GetFieldLength();
inline const units::meter_t fieldWidth = layout.GetFieldWidth();

// Horizontal half-width of the hub opening (used to find hub center)
static constexpr units::meter_t kHubHalfWidth = 23.5_in;  // 47 in wide / 2
static constexpr units::meter_t kHubHeight = 72.0_in;     // top of hub

// Blue hub center (tag 26 is the blue hub AprilTag)
inline frc::Translation2d GetHubBlue() {
  auto tag = layout.GetTagPose(26);
  units::meter_t x = tag.has_value() ? tag->X() : units::meter_t{2.0};
  return frc::Translation2d{x + kHubHalfWidth, fieldWidth / 2.0};
}

// Red hub center (tag 10 is the red hub AprilTag)
inline frc::Translation2d GetHubRed() {
  auto tag = layout.GetTagPose(10);
  units::meter_t x = tag.has_value() ? tag->X() : units::meter_t{14.5};
  return frc::Translation2d{x - kHubHalfWidth, fieldWidth / 2.0};
}
}  // namespace FieldConstants

namespace AimConstants {
// Drum shooter is mounted at the BACK of the robot.
// Positive X = forward on the robot, so the shooter is at -offset.
constexpr units::meter_t kShooterXOffset = -5.75_in;  // 5.75 in behind center

// Drivetrain heading tolerance to consider the robot "aimed"
constexpr units::degree_t kHeadingTolerance = 2.0_deg;
}  // namespace AimConstants

namespace ShooterConstants {
constexpr int kBottomRightShooterID = 23;  // X60 - CANivore
constexpr int kTopRightShooterID = 24;     // X60 - CANivore (active flywheel)
constexpr int kBottomLeftShooterID = 33;   // X60 - CANivore
constexpr int kTopLeftShooterID = 34;      // X60 - CANivore
constexpr int kHoodID = 54;                // Neo Vortex - RIO

// Hood gear reduction: 9:1 gearbox × (180T sector / 10T sprocket) = 162:1
constexpr double kHoodGearRatio = 9.0 * (180.0 / 10.0);  // 162.0

// Hood angle range (degrees at the pivot)
constexpr double kHoodMinDegrees = 15.0;  // lowest / default angle
constexpr double kHoodMaxDegrees = 45.0;  // highest angle
constexpr double kHoodToleranceDeg = 1.0;

// Hood PID (position control, slot 0) — tune kP if it overshoots
constexpr double kHoodP = 0.15;
constexpr double kHoodI = 0.0;
constexpr double kHoodD = 0.5;

constexpr const char* kCANivoreName = "Chassis";

// ── Flywheel (2× Kraken X60, IDs 24 & 34, CANivore, Phoenix Pro) ──────────
// Target velocity in rotations-per-second (RPS) for VelocityVoltage control.
constexpr double kFlywheelTargetRPS =
    2100.0 /
    60.0;  // 35.0 RPS — fallback only, interpolation map drives real shots

// Robot is considered "at speed" when within this many RPS of the setpoint.
constexpr double kFlywheelReadyToleranceRPS = 1.5;  // ±1.5 RPS (~±90 RPM)

// Idle speed — flywheel spins at this RPM when not actively shooting.
// Keeps the flywheel warm so it only needs to gain ~200 RPM on trigger pull
// instead of spinning from zero. Draws minimal current with 4 Krakens.
constexpr double kFlywheelIdleRPM = 1800.0;
constexpr double kFlywheelIdleRPS = kFlywheelIdleRPM / 60.0;  // 30.0 RPS

// Feed-forward gains for the Kraken X60 flywheel (slot 0, leader only).
// Using VelocityTorqueCurrentFOC — units are AMPS not volts.
// kS: amps to overcome static friction
// kV: amps per RPS of steady-state feedforward torque current.
//     Kraken X60 stall = ~366 A, free speed ~106 RPS → kV ≈ 366/106 ≈ 3.45.
//     Two motors share the load so effective kV per motor ≈ 1.2–1.5.
//     Set higher than theoretical so FF alone nearly holds speed under load.
// kP: amps per RPS of error — must be large enough to arrest a ball-impact
//     velocity dip (typically 10–15 RPS) within one or two control loops.
// kI: small wind-up to kill any residual steady-state error.
constexpr double kFlywheelKS = 0.15;
constexpr double kFlywheelKV = 0.22;  // trimmed from 0.25 — was still +50 RPM
constexpr double kFlywheelKA = 0.0;
constexpr double kFlywheelKP =
    6.0;  // doubled from 3.0 — ball impacts cause 400-500 RPM dip, need
          // aggressive recovery. 6.0 × 8.3 RPS error = 50A correction.
constexpr double kFlywheelKI =
    0.0;  // zeroed — was winding up and adding to overshoot

// Flywheel must be within tolerance for this long before the feeder is allowed
// to run. Prevents firing right as the wheel crosses the threshold.
constexpr double kFlywheelStableSeconds = 0.3;
}  // namespace ShooterConstants

namespace FeederConstants {
// CAN IDs
constexpr int kFeederID = 50;    // Neo Vortex - RIO
constexpr int kConveyorID = 51;  // Neo Vortex - RIO

// Speeds (RPM) — shared by both feeder and conveyor
constexpr double kFeedRPM =
    8000.0;  // feeding forward — increased for more power
constexpr double kOuttakeRPM = -3000.0;  // outtaking / unjamming

// Feeder velocity threshold to auto-enable conveyor (RPM)
constexpr double kFeederReadyThresholdRPM = 5500.0;

// PID (velocity control, slot 0) — shared by both motors
constexpr double kP = 0.05;
constexpr double kI = 0.00005;
constexpr double kD = 0.12;
constexpr double kFF = 0.00012;  // kV for Neo Vortex

// Unjam thresholds (shared for both motors)
constexpr double kJamCurrentThreshold = 35.0;    // amps — current spike
constexpr double kJamVelocityThreshold = 300.0;  // RPM — stall threshold
constexpr double kJamConfirmSec = 0.4;           // jam must persist this long
constexpr double kUnjamDurationSec = 0.4;        // length of reverse pulse
}  // namespace FeederConstants

namespace IntakeConstants {
// CAN IDs
constexpr int kRollerID = 52;  // Neo Vortex - RIO
constexpr int kDeployID = 53;  // Neo Vortex - RIO

// Deploy mechanism geometry
// Motor → 36:1 gearbox → 12T sprocket → 28T sprocket → intake pivot
// PositionConversionFactor = 360 / gearRatio, so encoder reports degrees
constexpr double kDeployGearRatio = 36.0 * (28.0 / 12.0);  // 84.0

// Deploy positions (degrees at the pivot)
constexpr double kDeployedAngleDeg = 0.0;    // down, starting position
constexpr double kStowedAngleDeg = 90.0;     // up
constexpr double kDeployToleranceDeg = 2.0;  // acceptable error

// Deploy PID (position control, slot 0) — input/output in degrees
constexpr double kDeployP = 0.015;
constexpr double kDeployI = 0.0;
constexpr double kDeployD = 0.08;

// Roller speeds (RPM)
constexpr double kRollerIntakeRPM = 3000.0;    // intaking
constexpr double kRollerEjectRPM = -3000.0;    // ejecting / unjamming
constexpr double kRollerOuttakeRPM = -3000.0;  // outtaking (deploy stays down)

// Roller PID (velocity control, slot 0)
constexpr double kRollerP = 0.00015;
constexpr double kRollerI = 0.0000001;
constexpr double kRollerD = 0.0001;
constexpr double kRollerFF =
    0.00177;  // FeedForwardConfig::kV() is in VOLTS per RPM (not duty cycle).
              // Neo Vortex free speed ~6784 RPM → 12V / 6784 ≈ 0.00177 V/RPM

// Unjam thresholds
constexpr double kJamCurrentThreshold =
    60.0;  // amps — must be a true stall, not spin-up
constexpr double kJamVelocityThreshold =
    1500.0;  // RPM — only trigger if nearly stopped (raised to avoid false
             // fires during spin-up)
constexpr double kJamConfirmSec = 0.5;     // jam must persist this long
constexpr double kUnjamDurationSec = 0.4;  // length of reverse pulse
}  // namespace IntakeConstants

namespace VisionConstants {
// ── Camera ────────────────────────────────────────────────────────────────
inline const std::string kLimelightName = "limelight-shooter";

// ── Rotation stddev ────────────────────────────────────────────────────────
// With 2+ tags MT1 rotation is reliable — use a real stddev so the estimator
// actually blends it. Single-tag rotation is still unreliable, pinned high.
constexpr double kRotStdDev = 9999.0;       // single-tag: ignore rotation
constexpr double kMultiTagRotStdDev = 5.0;  // multi-tag: trust rotation

// ── Tiered XY stddevs ─────────────────────────────────────────────────────
// Tier A: 2+ tags → tight constraint, update aggressively
constexpr double kMultiTagXYStdDev = 0.1;
// Tier B: 1 tag, large area (close + near odometry)
constexpr double kCloseTagXYStdDev = 0.7;
// Tier C: 1 tag, medium area (farther but still near odometry)
constexpr double kFarTagXYStdDev = 1.5;

// ── Tier thresholds ────────────────────────────────────────────────────────
// Minimum tag area (% of image) to consider a single-tag measurement
constexpr double kCloseTagAreaThresh = 0.8;  // Tier B: area > 0.8%
constexpr double kFarTagAreaThresh = 0.1;    // Tier C: area > 0.1%

// Maximum allowed pose jump from odometry for single-tag tiers
constexpr double kCloseTagMaxDeltaM = 0.5;  // Tier B: jump ≤ 0.5 m
constexpr double kFarTagMaxDeltaM = 0.3;    // Tier C: jump ≤ 0.3 m

// ── Hard rejection filters ─────────────────────────────────────────────────
constexpr double kMaxAcceptedDistM = 8.0;    // reject if avg tag dist > 8 m
constexpr double kAmbiguityThreshold = 0.3;  // reject if ambiguity > 0.3

// ── Gyro heading seed from vision ─────────────────────────────────────────
// Seeds the gyro when 2+ tags are visible with decent ambiguity. Relaxed
// speed threshold so it works during slow driving, not just when stopped.
constexpr double kHeadingSeedAmbiguityMax = 0.25;      // was 0.15
constexpr double kHeadingSeedMaxLinearSpeedMps = 0.5;  // was 0.05
constexpr double kHeadingSeedMaxOmegaRadps = 0.3;      // was 0.05
}  // namespace VisionConstants

namespace ShiftConstants {
// 2026 shift boundaries — match time remaining (seconds)
// >130: Transition (both hubs active)
// 105–130: Shift 1 | 80–105: Shift 2 | 55–80: Shift 3 | 30–55: Shift 4
// ≤30: Endgame (both hubs active)
constexpr double kShift1Start = 130.0;
constexpr double kShift2Start = 105.0;
constexpr double kShift3Start = 80.0;
constexpr double kShift4Start = 55.0;
constexpr double kEndgameStart = 30.0;

// Warning/rumble timing before each shift boundary
constexpr double kShiftWarningSeconds = 5.0;
constexpr double kRumbleWarningSeconds = 3.0;
}  // namespace ShiftConstants
