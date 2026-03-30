// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc2/command/SubsystemBase.h>

#include <array>
#include <functional>

// Forward-declare the drivetrain to avoid a circular header dependency.
namespace subsystems {
class CommandSwerveDrivetrain;
}

/**
 * VisionSubsystem
 *
 * Reads MegaTag1 pose estimates from a Limelight 4 named
 * "limelight-shooter" and feeds them into the swerve drivetrain's pose
 * estimator via AddVisionMeasurement().
 *
 * MegaTag1 notes:
 *   - SetRobotOrientation() must be called every loop with the current IMU
 *     yaw so the camera can resolve ambiguity correctly.
 *   - Rotation from MT1 is unreliable → rotation stddev is pinned to 9999.
 *   - XY stddev grows with distance and is penalised when only one tag is
 *     visible.
 *   - Measurements failing any rejection filter are silently dropped.
 *   - Odometry updates run unconditionally (enabled and disabled).
 *   - When 2+ tags are visible, ambiguity is low, and the robot is stationary,
 *     the gyro heading is seeded directly from the vision pose rotation.
 *
 * Usage (in RobotContainer):
 *   vision.SetDrivetrain(drivetrain);   // called once in constructor
 */
class VisionSubsystem : public frc2::SubsystemBase {
 public:
  VisionSubsystem();

  /**
   * Must be called once (e.g. at the end of RobotContainer's constructor)
   * to give the subsystem a reference to the drivetrain so it can push
   * measurements.
   */
  void SetDrivetrain(subsystems::CommandSwerveDrivetrain& dt);

  /**
   * Called automatically every robot loop by the scheduler.
   * Pulls the latest MegaTag1 estimate and, if valid, feeds it to the
   * drivetrain pose estimator.
   */
  void Periodic() override;

  // ── Public telemetry ──────────────────────────────────────────────────────

  /** Number of tags seen in the most recent accepted measurement. */
  int lastTagCount{0};

  /** Average tag distance of the most recent accepted measurement (metres). */
  double lastAvgTagDistM{0.0};

  /** XY stddev used in the most recent accepted measurement (metres). */
  double lastXYStdDev{0.0};

  /** Euclidean distance between the vision pose and odometry pose (metres). */
  double lastPoseDifferenceM{0.0};

  /** True if the last Periodic() call accepted and applied a measurement. */
  bool lastMeasurementAccepted{false};

  /** True if the last Periodic() call seeded the gyro heading from vision. */
  bool lastHeadingSeeded{false};

 private:
  /** Pointer to the drivetrain; set via SetDrivetrain(). Null-safe. */
  subsystems::CommandSwerveDrivetrain* m_drivetrain{nullptr};

  /**
   * Selects an XY stddev tier based on tag count, tag area, and pose
   * difference vs odometry (mirrors the 2024 tiered logic).
   * Returns {xyStdDev, xyStdDev, kRotStdDev}, or {-1,-1,-1} to signal
   * "reject" when no tier matches.
   */
  std::array<double, 3> CalcStdDevs(int tagCount, double avgTagArea,
                                    double poseDifferenceM);

  /**
   * Returns false if the measurement should be rejected before tier selection.
   *
   * Hard rejection criteria:
   *   1. tagCount == 0
   *   2. avgTagDist > kMaxAcceptedDistM
   *   3. any rawFiducial ambiguity > kAmbiguityThreshold
   */
  bool ShouldAccept(int tagCount, double avgDistM, double maxAmbiguity);
};
