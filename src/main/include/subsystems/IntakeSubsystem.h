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

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  // ── States ────────────────────────────────────────────────────────────────
  enum class DeployState {
    kDeployed,  // pivot at 0 deg (down)
    kStowed     // pivot at 90 deg (up)
  };

  enum class RollerState {
    kIdle,       // stopped
    kIntaking,   // running inward
    kOuttaking,  // all motors reversed, deploy stays down — feeds balls back
                 // out
    kEjecting,   // running outward (manual unjam override)
    kUnjamming   // auto-reverse pulse to clear a jam, then resumes intaking
  };

  IntakeSubsystem();

  // ── State setters ─────────────────────────────────────────────────────────
  /** Set the desired deploy state: kDeployed (0 deg) or kStowed (90 deg). */
  void SetDeployState(DeployState state);

  /** Set the deploy pivot to an arbitrary angle (degrees). Used for agitation.
   *  Does not change m_deployState so the subsystem can resume normally. */
  void SetTargetAngleDeg(double angleDeg);

  /** Set the desired roller state. kUnjamming is managed automatically by
   * Periodic. */
  void SetRollerState(RollerState state);

  // ── State accessors ───────────────────────────────────────────────────────
  DeployState GetDeployState() const { return m_deployState; }
  RollerState GetRollerState() const { return m_rollerState; }

  /** Current intake pivot angle in degrees. */
  double GetAngleDeg() const;

  /** True when deploy pivot is within tolerance of its target angle. */
  bool IsDeployAtTarget() const;

  // ── Periodic (drives motor outputs and unjam logic) ───────────────────────
  void Periodic() override;

 private:
  // ── Motors ────────────────────────────────────────────────────────────────
  rev::spark::SparkFlex m_deployMotor{
      IntakeConstants::kDeployID, rev::spark::SparkFlex::MotorType::kBrushless};
  rev::spark::SparkFlex m_rollerMotor{
      IntakeConstants::kRollerID, rev::spark::SparkFlex::MotorType::kBrushless};

  // ── Controllers / encoders ────────────────────────────────────────────────
  rev::spark::SparkClosedLoopController m_deployController =
      m_deployMotor.GetClosedLoopController();
  rev::spark::SparkRelativeEncoder m_deployEncoder = m_deployMotor.GetEncoder();

  rev::spark::SparkClosedLoopController m_rollerController =
      m_rollerMotor.GetClosedLoopController();

  // ── State ─────────────────────────────────────────────────────────────────
  DeployState m_deployState = DeployState::kDeployed;
  RollerState m_rollerState = RollerState::kIdle;
  double m_targetAngleDeg = IntakeConstants::kDeployedAngleDeg;

  // Jam detection: timer confirms a sustained jam before triggering unjam
  frc::Timer m_jamTimer;
  bool m_isJamming = false;

  // Unjam timer: times the reverse-pulse duration
  frc::Timer m_unjamTimer;

  // ── Private helpers ───────────────────────────────────────────────────────
  void ConfigureMotors();

  /** Returns true when both the current spike and velocity-stall conditions are
   * met. */
  bool IsJammed();
};
