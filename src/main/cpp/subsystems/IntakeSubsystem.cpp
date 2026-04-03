// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkFlexConfig.h>

#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

IntakeSubsystem::IntakeSubsystem() {
  ConfigureMotors();
  // Robot always starts deployed (down, 0 deg) — zero the encoder to match
  m_deployEncoder.SetPosition(IntakeConstants::kDeployedAngleDeg);
}

// ─────────────────────────────────────────────────────────────────────────────
// Motor configuration
// ─────────────────────────────────────────────────────────────────────────────

void IntakeSubsystem::ConfigureMotors() {
  // ── Deploy ────────────────────────────────────────────────────────────────
  rev::spark::SparkFlexConfig deployConfig{};

  deployConfig.SmartCurrentLimit(40, 20);
  deployConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  deployConfig.Inverted(false);
  deployConfig.VoltageCompensation(12.0);
  deployConfig.ClosedLoopRampRate(0.1);

  // PositionConversionFactor: encoder natively reports degrees
  deployConfig.encoder.PositionConversionFactor(
      360.0 / IntakeConstants::kDeployGearRatio);

  // Soft limits in degrees (consistent with PCF)
  deployConfig.softLimit.ForwardSoftLimit(IntakeConstants::kStowedAngleDeg)
      .ForwardSoftLimitEnabled(true)
      .ReverseSoftLimit(IntakeConstants::kDeployedAngleDeg)
      .ReverseSoftLimitEnabled(true);

  deployConfig.closedLoop.SetFeedbackSensor(rev::spark::kPrimaryEncoder)
      .Pid(IntakeConstants::kDeployP, IntakeConstants::kDeployI,
           IntakeConstants::kDeployD)
      .OutputRange(-0.8, 0.8);

  m_deployMotor.Configure(deployConfig, rev::ResetMode::kResetSafeParameters,
                          rev::PersistMode::kPersistParameters);

  // ── Rollers ───────────────────────────────────────────────────────────────
  rev::spark::SparkFlexConfig rollerConfig{};

  rollerConfig.SmartCurrentLimit(60);
  rollerConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
  rollerConfig.VoltageCompensation(12.0);

  rollerConfig.closedLoop.SetFeedbackSensor(rev::spark::kPrimaryEncoder)
      .Pid(IntakeConstants::kRollerP, IntakeConstants::kRollerI,
           IntakeConstants::kRollerD)
      .OutputRange(-1.0, 1.0);
  rollerConfig.closedLoop.feedForward.kV(IntakeConstants::kRollerFF);

  m_rollerMotor.Configure(rollerConfig, rev::ResetMode::kResetSafeParameters,
                          rev::PersistMode::kPersistParameters);
}

// ─────────────────────────────────────────────────────────────────────────────
// State setters
// ─────────────────────────────────────────────────────────────────────────────

void IntakeSubsystem::SetDeployState(DeployState state) {
  m_deployState = state;
  m_targetAngleDeg = (state == DeployState::kStowed)
                         ? IntakeConstants::kStowedAngleDeg
                         : IntakeConstants::kDeployedAngleDeg;
  // TEMPORARILY DISABLED — intake deploy chain broken
  m_deployController.SetSetpoint(
      m_targetAngleDeg, rev::spark::SparkLowLevel::ControlType::kPosition);
}

void IntakeSubsystem::SetTargetAngleDeg(double angleDeg) {
  // Clamp to valid range so we don't exceed soft limits
  angleDeg = std::clamp(angleDeg, IntakeConstants::kDeployedAngleDeg,
                        IntakeConstants::kStowedAngleDeg);
  m_targetAngleDeg = angleDeg;
  // TEMPORARILY DISABLED — intake deploy chain broken
  m_deployController.SetSetpoint(
      m_targetAngleDeg, rev::spark::SparkLowLevel::ControlType::kPosition);
}

void IntakeSubsystem::SetRollerState(RollerState state) {
  if (state == RollerState::kUnjamming) return;  // auto-managed only
  m_rollerState = state;

  switch (m_rollerState) {
    case RollerState::kIntaking:
      m_rollerController.SetSetpoint(
          IntakeConstants::kRollerIntakeRPM,
          rev::spark::SparkLowLevel::ControlType::kVelocity);
      break;
    case RollerState::kOuttaking:
      // Deploy stays wherever it is — just reverse the roller
      m_rollerController.SetSetpoint(
          IntakeConstants::kRollerOuttakeRPM,
          rev::spark::SparkLowLevel::ControlType::kVelocity);
      break;
    case RollerState::kEjecting:
      m_rollerController.SetSetpoint(
          IntakeConstants::kRollerEjectRPM,
          rev::spark::SparkLowLevel::ControlType::kVelocity);
      break;
    case RollerState::kIdle:
    default:
      m_rollerMotor.StopMotor();
      break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Accessors
// ─────────────────────────────────────────────────────────────────────────────

// Encoder natively reports degrees thanks to PositionConversionFactor
double IntakeSubsystem::GetAngleDeg() const {
  return m_deployEncoder.GetPosition();
}

bool IntakeSubsystem::IsDeployAtTarget() const {
  return std::abs(GetAngleDeg() - m_targetAngleDeg) <=
         IntakeConstants::kDeployToleranceDeg;
}

// ─────────────────────────────────────────────────────────────────────────────
// Jam detection — current spike AND velocity stall must both be true
// ─────────────────────────────────────────────────────────────────────────────

bool IntakeSubsystem::IsJammed() {
  return m_rollerMotor.GetOutputCurrent() >
             IntakeConstants::kJamCurrentThreshold &&
         std::abs(m_rollerMotor.GetEncoder().GetVelocity()) <
             IntakeConstants::kJamVelocityThreshold;
}

// ─────────────────────────────────────────────────────────────────────────────
// Brake / coast mode for field setup
// ─────────────────────────────────────────────────────────────────────────────

void IntakeSubsystem::SetDeployBrakeMode(bool brake) {
  rev::spark::SparkFlexConfig cfg{};
  cfg.SetIdleMode(brake ? rev::spark::SparkBaseConfig::IdleMode::kBrake
                        : rev::spark::SparkBaseConfig::IdleMode::kCoast);
  m_deployMotor.Configure(cfg, rev::ResetMode::kNoResetSafeParameters,
                          rev::PersistMode::kNoPersistParameters);
}

// ─────────────────────────────────────────────────────────────────────────────
// Periodic — only handles automatic unjam logic and telemetry
// ─────────────────────────────────────────────────────────────────────────────

void IntakeSubsystem::Periodic() {
  // ── Sustained jam detection ───────────────────────────────────────────────
  if (m_rollerState == RollerState::kIntaking) {
    if (IsJammed()) {
      if (!m_isJamming) {
        m_jamTimer.Restart();
        m_isJamming = true;
      }
      if (m_jamTimer.HasElapsed(
              units::second_t{IntakeConstants::kJamConfirmSec})) {
        m_isJamming = false;
        m_rollerState = RollerState::kUnjamming;
        m_unjamTimer.Restart();
        m_rollerController.SetSetpoint(
            IntakeConstants::kRollerEjectRPM,
            rev::spark::SparkLowLevel::ControlType::kVelocity);
      }
    } else {
      m_isJamming = false;
      m_jamTimer.Stop();
    }
  } else if (m_rollerState == RollerState::kUnjamming &&
             m_unjamTimer.HasElapsed(
                 units::second_t{IntakeConstants::kUnjamDurationSec})) {
    m_rollerState = RollerState::kIntaking;
    m_rollerController.SetSetpoint(
        IntakeConstants::kRollerIntakeRPM,
        rev::spark::SparkLowLevel::ControlType::kVelocity);
  }

  // ── Telemetry ─────────────────────────────────────────────────────────────
  frc::SmartDashboard::PutNumber("Intake/AngleDeg", GetAngleDeg());
  frc::SmartDashboard::PutNumber("Intake/TargetAngleDeg", m_targetAngleDeg);
  frc::SmartDashboard::PutNumber("Intake/RollerRPM",
                                 m_rollerMotor.GetEncoder().GetVelocity());
  frc::SmartDashboard::PutNumber("Intake/RollerCurrentA",
                                 m_rollerMotor.GetOutputCurrent());
  frc::SmartDashboard::PutBoolean("Intake/IsJammed", m_isJamming);
  frc::SmartDashboard::PutBoolean("Intake/AtTarget", IsDeployAtTarget());

  frc::SmartDashboard::PutString(
      "Intake/DeployState",
      m_deployState == DeployState::kStowed ? "Stowed" : "Deployed");
  frc::SmartDashboard::PutString(
      "Intake/RollerState",
      m_rollerState == RollerState::kIdle        ? "Idle"
      : m_rollerState == RollerState::kIntaking  ? "Intaking"
      : m_rollerState == RollerState::kOuttaking ? "Outtaking"
      : m_rollerState == RollerState::kEjecting  ? "Ejecting"
                                                 : "Unjamming");
}
