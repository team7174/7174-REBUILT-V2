// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/AimSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/math.h>

void AimSubsystem::UpdatePose(frc::Pose2d robotPose) {
  m_robotPose = robotPose;
}

void AimSubsystem::Periodic() {
  auto alliance = frc::DriverStation::GetAlliance();
  frc::Translation2d hub = GetTargetHub();

  // ── 0. Determine mode ────────────────────────────────────────────────────
  // "Past the hub" means the robot has crossed into opponent territory.
  // Blue alliance: robot X > hub X  (blue hub is at low X, field goes right)
  // Red  alliance: robot X < hub X  (red hub is at high X)
  bool isPastHub = false;
  if (alliance.has_value() &&
      alliance.value() == frc::DriverStation::Alliance::kRed) {
    isPastHub = m_robotPose.X() < hub.X();
  } else {
    isPastHub = m_robotPose.X() > hub.X();
  }
  currentMode = isPastHub ? AimMode::kPassingAlly : AimMode::kShootingAtHub;

  // ── 1. Resolve active target ─────────────────────────────────────────────
  frc::Translation2d target =
      (currentMode == AimMode::kPassingAlly) ? GetPassTarget() : hub;

  // ── 2. Distance & heading from shooter to target ─────────────────────────
  frc::Translation2d shooter = GetShooterPose();
  distance = shooter.Distance(target);

  // Angle from the shooter pointing directly AT the target.
  // The 180° flip (so the robot's back faces the target) is applied at the
  // drive request call site (WithTargetDirection), not here, so this value
  // can be reused cleanly for both the heading controller and IsAimed().
  units::degree_t angleToTarget =
      units::math::atan2(target.Y() - shooter.Y(), target.X() - shooter.X());

  robotHeading = angleToTarget;
  while (robotHeading.value() > 180.0)
    robotHeading = units::degree_t{robotHeading.value() - 360.0};
  while (robotHeading.value() < -180.0)
    robotHeading = units::degree_t{robotHeading.value() + 360.0};

  // ── 3. SmartDashboard telemetry ──────────────────────────────────────────
  frc::SmartDashboard::PutString(
      "Aim/Alliance", (alliance.has_value() &&
                       alliance.value() == frc::DriverStation::Alliance::kRed)
                          ? "RED"
                          : "BLUE");
  frc::SmartDashboard::PutString(
      "Aim/Mode",
      currentMode == AimMode::kShootingAtHub ? "SHOOTING" : "PASSING");
  frc::SmartDashboard::PutNumber("Aim/Distance (m)", distance.value());
  frc::SmartDashboard::PutNumber("Aim/Angle To Target (deg)",
                                 robotHeading.value());
  frc::SmartDashboard::PutNumber("Aim/Desired Back Heading (deg)",
                                 robotHeading.value() + 180.0);
  frc::SmartDashboard::PutNumber("Aim/Current Heading (deg)",
                                 m_robotPose.Rotation().Degrees().value());
  frc::SmartDashboard::PutNumber("Aim/Target X (m)", target.X().value());
  frc::SmartDashboard::PutNumber("Aim/Target Y (m)", target.Y().value());
  frc::SmartDashboard::PutBoolean("Aim/Is Aimed", IsAimed());
}

bool AimSubsystem::IsAimed() const {
  // The robot is "aimed" when the back of the robot faces the target, i.e.
  // the robot's current heading is within tolerance of (robotHeading + 180°).
  units::degree_t desiredBack = units::degree_t{robotHeading.value() + 180.0};
  units::degree_t delta = desiredBack - m_robotPose.Rotation().Degrees();
  while (delta.value() > 180.0) delta = units::degree_t{delta.value() - 360.0};
  while (delta.value() < -180.0) delta = units::degree_t{delta.value() + 360.0};
  return units::math::abs(delta) <= AimConstants::kHeadingTolerance;
}

frc::Translation2d AimSubsystem::GetTargetHub() const {
  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance.has_value() &&
      alliance.value() == frc::DriverStation::Alliance::kRed) {
    return FieldConstants::GetHubRed();
  }
  return FieldConstants::GetHubBlue();
}

frc::Translation2d AimSubsystem::GetShooterPose() const {
  units::meter_t sx =
      m_robotPose.X() + AimConstants::kShooterXOffset *
                            units::math::cos(m_robotPose.Rotation().Radians());
  units::meter_t sy =
      m_robotPose.Y() + AimConstants::kShooterXOffset *
                            units::math::sin(m_robotPose.Rotation().Radians());
  return frc::Translation2d{sx, sy};
}

frc::Translation2d AimSubsystem::GetPassTarget() const {
  // Fallback field dimensions if AprilTag layout hasn't loaded yet
  units::meter_t safeFieldLength = (FieldConstants::fieldLength < 10_m)
                                       ? 16.54_m
                                       : FieldConstants::fieldLength;

  auto alliance = frc::DriverStation::GetAlliance();
  units::meter_t passX;

  if (alliance.has_value() &&
      alliance.value() == frc::DriverStation::Alliance::kRed) {
    // Red alliance: hub is at high X, driver station wall is at fieldLength.
    // Pass target X = midpoint between red hub and red DS wall.
    passX = (FieldConstants::GetHubRed().X() + safeFieldLength) / 2.0;
  } else {
    // Blue alliance: hub is at low X, driver station wall is at X = 0.
    // Pass target X = midpoint between blue hub and X = 0.
    passX = FieldConstants::GetHubBlue().X() / 2.0;
  }

  return frc::Translation2d{passX, GetPassingZoneY()};
}

units::meter_t AimSubsystem::GetPassingZoneY() const {
  // Fallback field width if layout hasn't loaded yet
  units::meter_t safeFieldWidth =
      (FieldConstants::fieldWidth < 4_m) ? 8.21_m : FieldConstants::fieldWidth;

  units::meter_t centerY = safeFieldWidth / 2.0;

  // Two passing zones: bottom quarter (25%) and top quarter (75%) of field
  units::meter_t bottomZoneY = centerY / 2.0;
  units::meter_t topZoneY = centerY + (centerY / 2.0);

  // Aim at whichever zone the robot is currently closest to
  return (m_robotPose.Y() < centerY) ? bottomZoneY : topZoneY;
}
