// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/RelativeEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkFlex.h>

#include "Constants.h"

class FeederSubsystem : public frc2::SubsystemBase {
 public:
  // ── States ────────────────────────────────────────────────────────────────
  enum class FeederState {
    kIdle,       // both motors stopped
    kFeeding,    // feeder and conveyor both run at setpoint immediately
    kOuttaking,  // both motors reversed to push balls back out
    kUnjamming   // auto-managed: reverse pulse, then resume previous state
  };

  FeederSubsystem();

  // ── State setter ──────────────────────────────────────────────────────────
  /** Set the desired state. kUnjamming is managed automatically by Periodic. */
  void SetState(FeederState state);

  // ── State accessors ───────────────────────────────────────────────────────
  FeederState GetState() const { return m_state; }

  /** True when the feeder motor is at or above its feed setpoint velocity. */
  bool IsFeederAtSpeed();

  // ── Periodic ──────────────────────────────────────────────────────────────
  void Periodic() override;

 private:
  // ── Motors ────────────────────────────────────────────────────────────────
  rev::spark::SparkFlex m_feederMotor{
      FeederConstants::kFeederID, rev::spark::SparkFlex::MotorType::kBrushless};
  rev::spark::SparkFlex m_conveyorMotor{
      FeederConstants::kConveyorID,
      rev::spark::SparkFlex::MotorType::kBrushless};

  // ── Controllers / encoders ────────────────────────────────────────────────
  rev::spark::SparkClosedLoopController m_feederController =
      m_feederMotor.GetClosedLoopController();
  rev::spark::SparkClosedLoopController m_conveyorController =
      m_conveyorMotor.GetClosedLoopController();

  // ── State ─────────────────────────────────────────────────────────────────
  FeederState m_state = FeederState::kIdle;
  FeederState m_stateBeforeUnjam = FeederState::kIdle;  // resume after unjam
  bool m_conveyorActive = false;  // tracks whether conveyor has been enabled

  // Feeder jam detection
  frc::Timer m_feederJamTimer;
  bool m_feederJamming = false;

  // Conveyor jam detection
  frc::Timer m_conveyorJamTimer;
  bool m_conveyorJamming = false;

  // ── Unjam pulse timer
  frc::Timer m_unjamTimer;

  // ── Live PID tuning (SmartDashboard) ──────────────────────────────────────
  // Cached copies of the last values pushed to the motors; compared each loop
  // so we only re-configure when something actually changed.
  double m_tunedP = FeederConstants::kP;
  double m_tunedI = FeederConstants::kI;
  double m_tunedD = FeederConstants::kD;
  double m_tunedFF = FeederConstants::kFF;

  // ── Private helpers ───────────────────────────────────────────────────────
  void ConfigureMotors();

  bool IsFeederJammed();
  bool IsConveyorJammed();

  /** Read PID/FF from SmartDashboard and re-configure motors if any changed. */
  void UpdatePIDFromDashboard();
};
