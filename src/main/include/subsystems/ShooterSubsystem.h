// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkFlex.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/Follower.hpp"
#include "ctre/phoenix6/controls/VelocityTorqueCurrentFOC.hpp"

/**
 * ShooterSubsystem
 *
 * Drives two Kraken X60 flywheel motors (IDs 24 & 34, "Chassis" CANivore,
 * Phoenix Pro) at a fixed velocity setpoint using Phoenix 6 VelocityVoltage
 * closed-loop control. ID 34 (kTopLeftShooterID) follows ID 24 inverted via
 * StrictFollower so both wheels spin the ball in the same direction.
 *
 * Usage:
 *   shooter.SetRunning(true);   // start spinning at kFlywheelTargetRPS
 *   shooter.SetRunning(false);  // coast to stop
 *   shooter.IsAtSpeed();        // true when within kFlywheelReadyToleranceRPS
 */
class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  void SetRunning(bool run);

  /** Move hood to target angle in degrees, clamped to
   * kHoodMinDegrees–kHoodMaxDegrees. */
  void SetHoodAngle(double degrees);

  /** Current hood angle in degrees as reported by the encoder. */
  double GetHoodAngleDeg() const;

  bool IsAtSpeed();
  double GetVelocityRPS();

  /** True when the flywheel has been continuously at speed for at least
   *  kFlywheelStableSeconds — use this to gate the feeder so it doesn't
   *  fire right as the flywheel just crossed the threshold. */
  bool IsStableAtSpeed();

  void Periodic() override;

 private:
  // Leader — closed-loop runs here
  ctre::phoenix6::hardware::TalonFX m_flywheelLeader{
      ShooterConstants::kTopRightShooterID,
      ctre::phoenix6::CANBus{ShooterConstants::kCANivoreName}};

  // Follower — mirrors leader, inverted so both wheels push ball same way
  ctre::phoenix6::hardware::TalonFX m_flywheelFollower{
      ShooterConstants::kTopLeftShooterID,
      ctre::phoenix6::CANBus{ShooterConstants::kCANivoreName}};

  // TorqueCurrentFOC — immune to battery voltage sag, holds speed under load.
  // Requires Phoenix Pro license (already enabled on this robot).
  ctre::phoenix6::controls::VelocityTorqueCurrentFOC m_velocityRequest{0_tps};

  bool m_running{false};

  // Tracks how long the flywheel has been continuously within tolerance.
  // Feeder is only gated open after kFlywheelStableSeconds of being at speed.
  frc::Timer m_atSpeedTimer;
  bool m_wasAtSpeed{false};

  // Hood — Neo Vortex (SparkFlex, RIO CAN bus, ID 54). 162:1 reduction.
  // Position closed-loop holds angle between kHoodMinDegrees and
  // kHoodMaxDegrees.
  rev::spark::SparkFlex m_hoodMotor{
      ShooterConstants::kHoodID, rev::spark::SparkFlex::MotorType::kBrushless};
  rev::spark::SparkClosedLoopController m_hoodController =
      m_hoodMotor.GetClosedLoopController();
  rev::spark::SparkRelativeEncoder m_hoodEncoder = m_hoodMotor.GetEncoder();

  double m_targetHoodAngle = ShooterConstants::kHoodMinDegrees;

  void ConfigureMotors();
};
