// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkFlexConfig.h>

#include <algorithm>

using namespace ctre::phoenix6;

// ─────────────────────────────────────────────────────────────────────────────

ShooterSubsystem::ShooterSubsystem() {
  SetName("ShooterSubsystem");
  ConfigureMotors();
}

// ─────────────────────────────────────────────────────────────────────────────

void ShooterSubsystem::ConfigureMotors() {
  configs::TalonFXConfiguration cfg{};

  cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;

  // Slot 0 — VelocityTorqueCurrentFOC closed-loop on the leader.
  // Units are amps, not volts — kV/kP are amps per RPS.
  cfg.Slot0.kS = ShooterConstants::kFlywheelKS;
  cfg.Slot0.kV = ShooterConstants::kFlywheelKV;
  cfg.Slot0.kA = ShooterConstants::kFlywheelKA;
  cfg.Slot0.kP = ShooterConstants::kFlywheelKP;
  cfg.Slot0.kI = ShooterConstants::kFlywheelKI;
  cfg.Slot0.kD = 0.0;

  // Supply limit raised — 30A was starving TorqueCurrentFOC at steady state.
  // 45A × 2 motors = 90A total, acceptable for a flywheel holding speed.
  cfg.CurrentLimits.StatorCurrentLimit = 60.0_A;
  cfg.CurrentLimits.StatorCurrentLimitEnable = true;
  cfg.CurrentLimits.SupplyCurrentLimit = 45.0_A;
  cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

  // Apply same base config to both motors
  m_flywheelLeader.GetConfigurator().Apply(cfg);
  m_flywheelFollower.GetConfigurator().Apply(cfg);

  // Follower is mechanically inverted relative to the leader.
  // Use Follower (not StrictFollower) as it supports WithOpposeMasterDirection.
  m_flywheelFollower.SetControl(
      controls::Follower{ShooterConstants::kTopRightShooterID, true});

  m_velocityRequest.Slot = 0;
  // No EnableFOC field — VelocityTorqueCurrentFOC is always FOC by definition

  // ── Hood (SparkFlex, RIO CAN, ID 54) ─────────────────────────────────────
  // 9:1 gearbox × (180T sector / 10T sprocket) = 162:1 total reduction.
  // PositionConversionFactor = 360 / 162 → encoder reports hood degrees
  // directly.
  rev::spark::SparkFlexConfig hoodCfg{};
  hoodCfg.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  hoodCfg.SmartCurrentLimit(40);
  hoodCfg.Inverted(false);
  hoodCfg.VoltageCompensation(12.0);

  hoodCfg.encoder.PositionConversionFactor(360.0 /
                                           ShooterConstants::kHoodGearRatio);

  // Soft limits prevent driving past the physical range
  hoodCfg.softLimit.ForwardSoftLimit(ShooterConstants::kHoodMaxDegrees)
      .ForwardSoftLimitEnabled(true)
      .ReverseSoftLimit(ShooterConstants::kHoodMinDegrees)
      .ReverseSoftLimitEnabled(true);

  hoodCfg.closedLoop.SetFeedbackSensor(rev::spark::kPrimaryEncoder)
      .Pid(ShooterConstants::kHoodP, ShooterConstants::kHoodI,
           ShooterConstants::kHoodD)
      .OutputRange(-1.0, 1.0);

  m_hoodMotor.Configure(hoodCfg, rev::ResetMode::kResetSafeParameters,
                        rev::PersistMode::kPersistParameters);

  // Assume hood starts at min angle on boot; zero encoder to match
  m_hoodEncoder.SetPosition(ShooterConstants::kHoodMinDegrees);

  // Command min angle immediately
  m_hoodController.SetSetpoint(
      ShooterConstants::kHoodMinDegrees,
      rev::spark::SparkLowLevel::ControlType::kPosition);

  // Pre-populate SmartDashboard tuning entry (only sets value if key doesn't
  // exist yet)
  frc::SmartDashboard::SetDefaultNumber("Shooter/HoodTuneDeg",
                                        ShooterConstants::kHoodMinDegrees);
}

// ─────────────────────────────────────────────────────────────────────────────

void ShooterSubsystem::SetHoodAngle(double degrees) {
  m_targetHoodAngle = std::clamp(degrees, ShooterConstants::kHoodMinDegrees,
                                 ShooterConstants::kHoodMaxDegrees);
  m_hoodController.SetSetpoint(
      m_targetHoodAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
}

double ShooterSubsystem::GetHoodAngleDeg() const {
  return m_hoodEncoder.GetPosition();
}

// ─────────────────────────────────────────────────────────────────────────────

void ShooterSubsystem::SetRunning(bool run) {
  m_running = run;
  if (m_running) {
    m_flywheelLeader.SetControl(m_velocityRequest.WithVelocity(
        units::turns_per_second_t{ShooterConstants::kFlywheelTargetRPS}));
  } else {
    m_flywheelLeader.SetControl(controls::NeutralOut{});
    // Follower will coast automatically when leader is neutral
  }
}

// ─────────────────────────────────────────────────────────────────────────────

bool ShooterSubsystem::IsAtSpeed() {
  if (!m_running) return false;
  double velocity = m_flywheelLeader.GetVelocity().GetValueAsDouble();
  double error = ShooterConstants::kFlywheelTargetRPS - velocity;
  return std::abs(error) <= ShooterConstants::kFlywheelReadyToleranceRPS;
}

bool ShooterSubsystem::IsStableAtSpeed() {
  return IsAtSpeed() && m_atSpeedTimer.HasElapsed(units::second_t{
                            ShooterConstants::kFlywheelStableSeconds});
}

double ShooterSubsystem::GetVelocityRPS() {
  return m_flywheelLeader.GetVelocity().GetValueAsDouble();
}

// ─────────────────────────────────────────────────────────────────────────────

void ShooterSubsystem::Periodic() {
  // Track how long we've been continuously at speed for stable-gate logic
  bool atSpeed = IsAtSpeed();
  if (atSpeed && !m_wasAtSpeed) {
    m_atSpeedTimer.Restart();
  } else if (!atSpeed) {
    m_atSpeedTimer.Stop();
    m_atSpeedTimer.Reset();
  }
  m_wasAtSpeed = atSpeed;

  frc::SmartDashboard::PutNumber("Shooter/VelocityRPS", GetVelocityRPS());
  frc::SmartDashboard::PutNumber("Shooter/VelocityRPM",
                                 GetVelocityRPS() * 60.0);
  frc::SmartDashboard::PutBoolean("Shooter/AtSpeed", atSpeed);
  frc::SmartDashboard::PutBoolean("Shooter/StableAtSpeed", IsStableAtSpeed());
  frc::SmartDashboard::PutBoolean("Shooter/Running", m_running);
  frc::SmartDashboard::PutNumber("Shooter/TargetRPS",
                                 ShooterConstants::kFlywheelTargetRPS);
  frc::SmartDashboard::PutNumber(
      "Shooter/LeaderCurrentA",
      m_flywheelLeader.GetStatorCurrent().GetValueAsDouble());
  frc::SmartDashboard::PutNumber(
      "Shooter/FollowerCurrentA",
      m_flywheelFollower.GetStatorCurrent().GetValueAsDouble());

  // Continuously re-send the current target to the controller every loop
  m_hoodController.SetSetpoint(
      m_targetHoodAngle, rev::spark::SparkLowLevel::ControlType::kPosition);

  frc::SmartDashboard::PutNumber("Shooter/HoodAngleDeg", GetHoodAngleDeg());
  frc::SmartDashboard::PutNumber("Shooter/HoodTargetDeg", m_targetHoodAngle);
  frc::SmartDashboard::PutBoolean(
      "Shooter/HoodAtTarget", std::abs(GetHoodAngleDeg() - m_targetHoodAngle) <=
                                  ShooterConstants::kHoodToleranceDeg);
}
