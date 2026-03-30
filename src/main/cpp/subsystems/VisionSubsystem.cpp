// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/time.h>

#include <cmath>

#include "Constants.h"
#include "LimelightHelpers.h"
#include "subsystems/CommandSwerveDrivetrain.h"

// ─────────────────────────────────────────────────────────────────────────────

VisionSubsystem::VisionSubsystem() { SetName("VisionSubsystem"); }

void VisionSubsystem::SetDrivetrain(subsystems::CommandSwerveDrivetrain& dt) {
  m_drivetrain = &dt;
}

// ─────────────────────────────────────────────────────────────────────────────

void VisionSubsystem::Periodic() {
  // NOTE: Periodic() is called unconditionally by the CommandScheduler
  // (via Robot::RobotPeriodic) whether the robot is enabled or disabled.
  // Odometry is therefore always kept up-to-date with vision.

  // ── 1. Seed robot orientation so MT1 can resolve ambiguity ───────────────
  // Must be called every loop before getBotPoseEstimate_wpiBlue.
  double currentYawDeg = 0.0;
  if (m_drivetrain) {
    currentYawDeg = m_drivetrain->GetState().Pose.Rotation().Degrees().value();
  }
  LimelightHelpers::SetRobotOrientation(VisionConstants::kLimelightName,
                                        currentYawDeg,
                                        0.0,   // yaw rate — not needed for MT1
                                        0.0,   // pitch
                                        0.0,   // pitch rate
                                        0.0,   // roll
                                        0.0);  // roll rate

  // ── 2. Pull MegaTag1 estimate (WPILib Blue-origin) ────────────────────────
  LimelightHelpers::PoseEstimate mt1 =
      LimelightHelpers::getBotPoseEstimate_wpiBlue(
          VisionConstants::kLimelightName);

  // ── 3. Validity gate — rawFiducials empty means no real data this frame ──
  // validPoseEstimate() returns false when rawFiducials is empty, which
  // happens when the NT entry hasn't updated or the camera sees nothing.
  if (!LimelightHelpers::validPoseEstimate(mt1)) {
    frc::SmartDashboard::PutBoolean("Vision/Accepted", false);
    frc::SmartDashboard::PutString("Vision/RejectReason", "no valid estimate");
    frc::SmartDashboard::PutBoolean("Vision/HeadingSeeded", false);
    lastMeasurementAccepted = false;
    lastHeadingSeeded = false;
    return;
  }

  // ── 4. Worst-case ambiguity across all rawFiducials ───────────────────────
  double maxAmbiguity = 0.0;
  for (const auto& fid : mt1.rawFiducials) {
    if (fid.ambiguity > maxAmbiguity) {
      maxAmbiguity = fid.ambiguity;
    }
  }

  // ── 5. Hard rejection filters ─────────────────────────────────────────────
  bool hardAccept = ShouldAccept(mt1.tagCount, mt1.avgTagDist, maxAmbiguity);

  // ── 6. Compute pose difference from current odometry ─────────────────────
  double poseDifferenceM = 0.0;
  if (m_drivetrain) {
    frc::Pose2d current = m_drivetrain->GetState().Pose;
    double dx = (mt1.pose.X() - current.X()).value();
    double dy = (mt1.pose.Y() - current.Y()).value();
    poseDifferenceM = std::sqrt(dx * dx + dy * dy);
  }

  // ── 7. Tier-based stddev selection ────────────────────────────────────────
  std::array<double, 3> stddevs{-1.0, -1.0, -1.0};
  if (hardAccept) {
    stddevs = CalcStdDevs(mt1.tagCount, mt1.avgTagArea, poseDifferenceM);
  }
  bool accepted = hardAccept && (stddevs[0] > 0.0);

  // ── 8. SmartDashboard telemetry ───────────────────────────────────────────
  frc::SmartDashboard::PutNumber("Vision/TagCount", mt1.tagCount);
  frc::SmartDashboard::PutNumber("Vision/AvgTagDist (m)", mt1.avgTagDist);
  frc::SmartDashboard::PutNumber("Vision/AvgTagArea (%)", mt1.avgTagArea);
  frc::SmartDashboard::PutNumber("Vision/MaxAmbiguity", maxAmbiguity);
  frc::SmartDashboard::PutNumber("Vision/PoseDiff (m)", poseDifferenceM);
  frc::SmartDashboard::PutBoolean("Vision/Accepted", accepted);
  frc::SmartDashboard::PutNumber("Vision/Pose X (m)", mt1.pose.X().value());
  frc::SmartDashboard::PutNumber("Vision/Pose Y (m)", mt1.pose.Y().value());
  frc::SmartDashboard::PutNumber("Vision/Timestamp (s)",
                                 mt1.timestampSeconds.value());

  // Log which filter is rejecting so we can debug on SmartDashboard
  if (!hardAccept) {
    if (mt1.tagCount == 0)
      frc::SmartDashboard::PutString("Vision/RejectReason", "tagCount==0");
    else if (mt1.avgTagDist > VisionConstants::kMaxAcceptedDistM)
      frc::SmartDashboard::PutString("Vision/RejectReason", "dist too large");
    else if (maxAmbiguity > VisionConstants::kAmbiguityThreshold)
      frc::SmartDashboard::PutString("Vision/RejectReason", "high ambiguity");
  } else if (!accepted) {
    frc::SmartDashboard::PutString("Vision/RejectReason", "no tier match");
  } else {
    frc::SmartDashboard::PutString("Vision/RejectReason", "none");
  }

  if (!accepted) {
    lastMeasurementAccepted = false;
    lastHeadingSeeded = false;
    frc::SmartDashboard::PutBoolean("Vision/HeadingSeeded", false);
    return;
  }

  // ── 9. Push to drivetrain pose estimator ─────────────────────────────────
  m_drivetrain->AddVisionMeasurement(mt1.pose, mt1.timestampSeconds, stddevs);

  // ── 10. Gyro heading seed ─────────────────────────────────────────────────
  // Conditions: 2+ tags, ambiguity below the tighter heading threshold,
  // and the robot is essentially stationary (very low linear and angular
  // speed). When all three hold, vision's rotation is reliable enough to
  // reset the gyro heading directly, correcting any accumulated IMU drift.
  lastHeadingSeeded = false;
  if (m_drivetrain && mt1.tagCount >= 2 &&
      maxAmbiguity < VisionConstants::kHeadingSeedAmbiguityMax) {
    frc::ChassisSpeeds speeds = m_drivetrain->GetState().Speeds;
    double linearSpeedMps = std::sqrt(speeds.vx.value() * speeds.vx.value() +
                                      speeds.vy.value() * speeds.vy.value());
    double omegaRadps = std::abs(speeds.omega.value());

    if (linearSpeedMps < VisionConstants::kHeadingSeedMaxLinearSpeedMps &&
        omegaRadps < VisionConstants::kHeadingSeedMaxOmegaRadps) {
      m_drivetrain->SeedHeadingFromVision(mt1.pose.Rotation());
      lastHeadingSeeded = true;
    }
  }
  frc::SmartDashboard::PutBoolean("Vision/HeadingSeeded", lastHeadingSeeded);

  // ── 11. Cache telemetry ───────────────────────────────────────────────────
  lastTagCount = mt1.tagCount;
  lastAvgTagDistM = mt1.avgTagDist;
  lastXYStdDev = stddevs[0];
  lastPoseDifferenceM = poseDifferenceM;
  lastMeasurementAccepted = true;

  frc::SmartDashboard::PutNumber("Vision/XY StdDev (m)", stddevs[0]);
}

// ─────────────────────────────────────────────────────────────────────────────

std::array<double, 3> VisionSubsystem::CalcStdDevs(int tagCount,
                                                   double avgTagArea,
                                                   double poseDifferenceM) {
  constexpr double kReject = -1.0;  // sentinel: drop this measurement

  // ── Tier A: multiple tags visible ─────────────────────────────────────────
  // Geometry is well-constrained — use the tightest stddev unconditionally.
  if (tagCount >= 2) {
    return {VisionConstants::kMultiTagXYStdDev,
            VisionConstants::kMultiTagXYStdDev, VisionConstants::kRotStdDev};
  }

  // ── Tier B: single tag, large area, close to odometry ────────────────────
  // Tag occupies a big portion of the image → camera is close → reliable.
  if (avgTagArea > VisionConstants::kCloseTagAreaThresh &&
      poseDifferenceM < VisionConstants::kCloseTagMaxDeltaM) {
    return {VisionConstants::kCloseTagXYStdDev,
            VisionConstants::kCloseTagXYStdDev, VisionConstants::kRotStdDev};
  }

  // ── Tier C: single tag, medium area, reasonably close to odometry ─────────
  if (avgTagArea > VisionConstants::kFarTagAreaThresh &&
      poseDifferenceM < VisionConstants::kFarTagMaxDeltaM) {
    return {VisionConstants::kFarTagXYStdDev, VisionConstants::kFarTagXYStdDev,
            VisionConstants::kRotStdDev};
  }

  // No tier matched → reject.
  return {kReject, kReject, kReject};
}

// ─────────────────────────────────────────────────────────────────────────────

bool VisionSubsystem::ShouldAccept(int tagCount, double avgDistM,
                                   double maxAmbiguity) {
  // No targets detected.
  if (tagCount == 0) return false;

  // Tags are too far away to trust the pose geometry.
  if (avgDistM > VisionConstants::kMaxAcceptedDistM) return false;

  // High ambiguity means two pose solutions are nearly equal in reprojection
  // error — the camera can't tell which is correct.
  if (maxAmbiguity > VisionConstants::kAmbiguityThreshold) return false;

  return true;
}
