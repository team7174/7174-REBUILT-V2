// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DriverStation.h>
#include <frc2/command/SubsystemBase.h>

#include <string>

#include "Constants.h"

// Reads FMS game data ('R'/'B') and match time to track alliance shift state.
// Determines if our hub is active, and provides shift warning/rumble signals.
class AllianceShiftSubsystem : public frc2::SubsystemBase {
 public:
  AllianceShiftSubsystem();
  void Periodic() override;

  bool IsOurHubActive() const { return m_ourHubActive; }
  bool IsShiftWarning() const { return m_shiftWarning; }
  bool IsShiftRumble() const { return m_shiftRumble; }
  int GetCurrentShift() const { return m_currentShift; }
  double GetMatchTimeRemaining() const { return m_matchTime; }
  double GetShiftTimeRemaining() const { return m_shiftTimeRemaining; }
  bool HasGameData() const { return m_gameDataReceived; }

 private:
  bool m_ourHubActive = true;
  bool m_shiftWarning = false;
  bool m_shiftRumble = false;
  int m_currentShift = 0;
  double m_matchTime = -1.0;
  double m_shiftTimeRemaining = 0.0;

  bool m_gameDataReceived = false;
  bool m_shift1Active = true;  // Is our hub active during Shifts 1 & 3?

  void UpdateShiftState();
};
