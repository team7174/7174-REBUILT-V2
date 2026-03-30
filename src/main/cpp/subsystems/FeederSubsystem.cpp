// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/FeederSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkFlexConfig.h>

#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

FeederSubsystem::FeederSubsystem() { ConfigureMotors(); }

// ─────────────────────────────────────────────────────────────────────────────
// Motor configuration
// ─────────────────────────────────────────────────────────────────────────────

void FeederSubsystem::ConfigureMotors() {
  // ── Feeder ────────────────────────────────────────────────────────────────
  rev::spark::SparkFlexConfig feederConfig{};

  feederConfig.Inverted(true);  // invert so positive RPM = feed direction
  feederConfig.SmartCurrentLimit(40, 20);
  feederConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
  feederConfig.VoltageCompensation(12.0);

  feederConfig.closedLoop.SetFeedbackSensor(rev::spark::kPrimaryEncoder)
      .Pid(FeederConstants::kP, FeederConstants::kI, FeederConstants::kD)
      .OutputRange(-1.0, 1.0);
  feederConfig.closedLoop.feedForward.kV(FeederConstants::kFF);

  m_feederMotor.Configure(feederConfig, rev::ResetMode::kResetSafeParameters,
                          rev::PersistMode::kPersistParameters);

  // ── Conveyor ──────────────────────────────────────────────────────────────
  rev::spark::SparkFlexConfig conveyorConfig{};

  conveyorConfig.Inverted(true);  // invert so positive RPM = feed direction
  conveyorConfig.SmartCurrentLimit(40, 20);
  conveyorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
  conveyorConfig.VoltageCompensation(12.0);

  conveyorConfig.closedLoop.SetFeedbackSensor(rev::spark::kPrimaryEncoder)
      .Pid(FeederConstants::kP, FeederConstants::kI, FeederConstants::kD)
      .OutputRange(-1.0, 1.0);
  conveyorConfig.closedLoop.feedForward.kV(FeederConstants::kFF);

  m_conveyorMotor.Configure(conveyorConfig,
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
}

// ─────────────────────────────────────────────────────────────────────────────
// State setter
// ─────────────────────────────────────────────────────────────────────────────

void FeederSubsystem::SetState(FeederState state) {
  if (state == FeederState::kUnjamming) return;  // auto-managed only
  if (state == m_state) return;

  m_state = state;
  m_conveyorActive = false;
  m_feederJamming = false;
  m_conveyorJamming = false;

  switch (m_state) {
    case FeederState::kFeeding:
      // Feeder starts immediately; conveyor waits for feeder to reach speed
      m_feederController.SetSetpoint(
          FeederConstants::kFeedRPM,
          rev::spark::SparkLowLevel::ControlType::kVelocity);
      m_conveyorMotor.StopMotor();
      break;

    case FeederState::kOuttaking:
      m_feederController.SetSetpoint(
          FeederConstants::kOuttakeRPM,
          rev::spark::SparkLowLevel::ControlType::kVelocity);
      m_conveyorController.SetSetpoint(
          FeederConstants::kOuttakeRPM,
          rev::spark::SparkLowLevel::ControlType::kVelocity);
      m_conveyorActive = true;
      break;

    case FeederState::kIdle:
    default:
      m_feederMotor.StopMotor();
      m_conveyorMotor.StopMotor();
      break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Accessors
// ─────────────────────────────────────────────────────────────────────────────

bool FeederSubsystem::IsFeederAtSpeed() {
  return m_feederMotor.GetEncoder().GetVelocity() >=
         FeederConstants::kFeederReadyThresholdRPM;
}

// ─────────────────────────────────────────────────────────────────────────────
// Jam detection — current spike AND velocity stall must both be true
// ─────────────────────────────────────────────────────────────────────────────

bool FeederSubsystem::IsFeederJammed() {
  return m_feederMotor.GetOutputCurrent() >
             FeederConstants::kJamCurrentThreshold &&
         std::abs(m_feederMotor.GetEncoder().GetVelocity()) <
             FeederConstants::kJamVelocityThreshold;
}

bool FeederSubsystem::IsConveyorJammed() {
  return m_conveyorMotor.GetOutputCurrent() >
             FeederConstants::kJamCurrentThreshold &&
         std::abs(m_conveyorMotor.GetEncoder().GetVelocity()) <
             FeederConstants::kJamVelocityThreshold;
}

// ─────────────────────────────────────────────────────────────────────────────
// Periodic
// Responsibilities:
//   1. Auto-enable conveyor once feeder reaches threshold (kFeeding only)
//   2. Sustained jam detection on both motors → unjam pulse → resume
// ─────────────────────────────────────────────────────────────────────────────

void FeederSubsystem::Periodic() {
  // ── 1. Auto-enable conveyor when feeder is up to speed ────────────────────
  if (m_state == FeederState::kFeeding && !m_conveyorActive &&
      IsFeederAtSpeed()) {
    m_conveyorController.SetSetpoint(
        FeederConstants::kFeedRPM,
        rev::spark::SparkLowLevel::ControlType::kVelocity);
    m_conveyorActive = true;
  }

  // ── 2. Jam detection (only while feeding or outtaking) ────────────────────
  if (m_state == FeederState::kFeeding || m_state == FeederState::kOuttaking) {
    // Feeder jam
    if (IsFeederJammed()) {
      if (!m_feederJamming) {
        m_feederJamTimer.Restart();
        m_feederJamming = true;
      }
      if (m_feederJamTimer.HasElapsed(
              units::second_t{FeederConstants::kJamConfirmSec})) {
        m_feederJamming = false;
        m_stateBeforeUnjam = m_state;
        m_state = FeederState::kUnjamming;
        m_unjamTimer.Restart();
        m_feederController.SetSetpoint(
            FeederConstants::kOuttakeRPM,
            rev::spark::SparkLowLevel::ControlType::kVelocity);
        m_conveyorController.SetSetpoint(
            FeederConstants::kOuttakeRPM,
            rev::spark::SparkLowLevel::ControlType::kVelocity);
      }
    } else {
      m_feederJamming = false;
      m_feederJamTimer.Stop();
    }

    // Conveyor jam (only if it's actively running)
    if (m_conveyorActive && IsConveyorJammed()) {
      if (!m_conveyorJamming) {
        m_conveyorJamTimer.Restart();
        m_conveyorJamming = true;
      }
      if (m_conveyorJamTimer.HasElapsed(
              units::second_t{FeederConstants::kJamConfirmSec})) {
        m_conveyorJamming = false;
        m_stateBeforeUnjam = m_state;
        m_state = FeederState::kUnjamming;
        m_unjamTimer.Restart();
        m_feederController.SetSetpoint(
            FeederConstants::kOuttakeRPM,
            rev::spark::SparkLowLevel::ControlType::kVelocity);
        m_conveyorController.SetSetpoint(
            FeederConstants::kOuttakeRPM,
            rev::spark::SparkLowLevel::ControlType::kVelocity);
      }
    } else {
      m_conveyorJamming = false;
      m_conveyorJamTimer.Stop();
    }
  }

  // ── 3. Unjam pulse expiry → resume previous state ─────────────────────────
  if (m_state == FeederState::kUnjamming &&
      m_unjamTimer.HasElapsed(
          units::second_t{FeederConstants::kUnjamDurationSec})) {
    SetState(m_stateBeforeUnjam);
  }

  // ── 4. Telemetry ──────────────────────────────────────────────────────────
  frc::SmartDashboard::PutNumber("Feeder/FeederRPM",
                                 m_feederMotor.GetEncoder().GetVelocity());
  frc::SmartDashboard::PutNumber("Feeder/ConveyorRPM",
                                 m_conveyorMotor.GetEncoder().GetVelocity());
  frc::SmartDashboard::PutNumber("Feeder/FeederCurrentA",
                                 m_feederMotor.GetOutputCurrent());
  frc::SmartDashboard::PutNumber("Feeder/ConveyorCurrentA",
                                 m_conveyorMotor.GetOutputCurrent());
  frc::SmartDashboard::PutBoolean("Feeder/AtSpeed", IsFeederAtSpeed());
  frc::SmartDashboard::PutBoolean("Feeder/ConveyorActive", m_conveyorActive);
  frc::SmartDashboard::PutBoolean("Feeder/FeederJammed", m_feederJamming);
  frc::SmartDashboard::PutBoolean("Feeder/ConveyorJammed", m_conveyorJamming);

  frc::SmartDashboard::PutString(
      "Feeder/State", m_state == FeederState::kIdle        ? "Idle"
                      : m_state == FeederState::kFeeding   ? "Feeding"
                      : m_state == FeederState::kOuttaking ? "Outtaking"
                                                           : "Unjamming");

  // ── 5. Live PID tuning ────────────────────────────────────────────────────
  UpdatePIDFromDashboard();
}

// ─────────────────────────────────────────────────────────────────────────────
// Live PID tuning
// ─────────────────────────────────────────────────────────────────────────────

void FeederSubsystem::UpdatePIDFromDashboard() {
  // Seed default values once (first call); subsequent calls read user edits
  frc::SmartDashboard::SetDefaultNumber("Feeder/Tune/kP", m_tunedP);
  frc::SmartDashboard::SetDefaultNumber("Feeder/Tune/kI", m_tunedI);
  frc::SmartDashboard::SetDefaultNumber("Feeder/Tune/kD", m_tunedD);
  frc::SmartDashboard::SetDefaultNumber("Feeder/Tune/kFF", m_tunedFF);

  double p = frc::SmartDashboard::GetNumber("Feeder/Tune/kP", m_tunedP);
  double i = frc::SmartDashboard::GetNumber("Feeder/Tune/kI", m_tunedI);
  double d = frc::SmartDashboard::GetNumber("Feeder/Tune/kD", m_tunedD);
  double ff = frc::SmartDashboard::GetNumber("Feeder/Tune/kFF", m_tunedFF);

  if (p == m_tunedP && i == m_tunedI && d == m_tunedD && ff == m_tunedFF) {
    return;  // nothing changed — skip re-configure
  }

  m_tunedP = p;
  m_tunedI = i;
  m_tunedD = d;
  m_tunedFF = ff;

  // Re-apply only the closed-loop gains; leave all other settings untouched.
  // kNoPersistParameters keeps the SPARK Flex flash wear-free during tuning.
  rev::spark::SparkFlexConfig cfg{};
  cfg.closedLoop.SetFeedbackSensor(rev::spark::kPrimaryEncoder)
      .Pid(p, i, d)
      .OutputRange(-1.0, 1.0);
  cfg.closedLoop.feedForward.kV(ff);

  m_feederMotor.Configure(cfg, rev::ResetMode::kNoResetSafeParameters,
                          rev::PersistMode::kNoPersistParameters);
  m_conveyorMotor.Configure(cfg, rev::ResetMode::kNoResetSafeParameters,
                            rev::PersistMode::kNoPersistParameters);
}
