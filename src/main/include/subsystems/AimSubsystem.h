// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/length.h>

#include "Constants.h"

// Calculates the field-relative robot heading and distance to either the
// alliance hub (shooting mode) or a passing zone target (passing mode).
//
// Mode is determined automatically each loop:
//   kShootingAtHub  — robot is on our side of the hub → aim at the hub
//   kPassingAlly    — robot has crossed past the hub into opponent territory
//                     → aim at the nearest passing zone on our side
//
// No turret — the robot itself must rotate so the back-mounted shooter faces
// the chosen target.  Feed robotPose every loop via UpdatePose(), then read
// robotHeading, distance, and currentMode as outputs.
class AimSubsystem : public frc2::SubsystemBase {
 public:
  enum class AimMode {
    kShootingAtHub,  // Aiming at the alliance scoring hub
    kPassingAlly     // Passing to ally in the mid-zone
  };

  AimSubsystem() = default;
  void Periodic() override;

  // Feed the latest robot pose from odometry/pose estimator each loop.
  void UpdatePose(frc::Pose2d robotPose);

  // ── Outputs (updated every Periodic loop) ────────────────────────────────

  // Field-relative angle pointing FROM the shooter TOWARD the active target.
  // To make the BACK of the robot face the target, pass (robotHeading + 180°)
  // to your swerve heading controller (FieldCentricFacingAngle).
  units::degree_t robotHeading{0_deg};

  // Distance from the shooter to the active target (metres).
  units::meter_t distance{0_m};

  // Current aiming mode — check this to decide flywheel/hood preset.
  AimMode currentMode{AimMode::kShootingAtHub};

  // True when the robot heading error is within
  // AimConstants::kHeadingTolerance.
  bool IsAimed() const;

  // True when the robot is in passing mode (past the hub).
  bool IsPassingMode() const { return currentMode == AimMode::kPassingAlly; }

 private:
  frc::Pose2d m_robotPose;

  // Returns the XY hub position for the current alliance.
  frc::Translation2d GetTargetHub() const;

  // Returns the XY position of the shooter (kShooterXOffset behind center).
  frc::Translation2d GetShooterPose() const;

  // Returns the XY passing zone target (midpoint between hub and DS wall,
  // top or bottom half depending on robot Y position).
  frc::Translation2d GetPassTarget() const;

  // Returns the Y coordinate of the passing zone the robot is closest to
  // (25% or 75% of field width).
  units::meter_t GetPassingZoneY() const;
};
