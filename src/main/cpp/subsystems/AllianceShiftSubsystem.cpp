// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/AllianceShiftSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

AllianceShiftSubsystem::AllianceShiftSubsystem() {}

// ─────────────────────────────────────────────────────────────────────────────
// Periodic
// ─────────────────────────────────────────────────────────────────────────────

void AllianceShiftSubsystem::Periodic() {
  // During auto or disabled: hub always active, reset state
  if (!frc::DriverStation::IsTeleopEnabled()) {
    m_currentShift = 0;
    m_ourHubActive = true;
    m_shiftWarning = false;
    m_shiftRumble = false;
    m_matchTime = -1.0;
    m_gameDataReceived = false;
    m_shift1Active = true;
  } else {
    // Teleop: parse game data on first valid message
    if (!m_gameDataReceived) {
      std::string gameData = frc::DriverStation::GetGameSpecificMessage();
      if (!gameData.empty()) {
        auto alliance = frc::DriverStation::GetAlliance();
        if (alliance.has_value()) {
          bool redInactiveFirst = false;
          switch (gameData[0]) {
            case 'R':
              redInactiveFirst = true;
              m_gameDataReceived = true;
              break;
            case 'B':
              redInactiveFirst = false;
              m_gameDataReceived = true;
              break;
            default:
              break;
          }

          if (m_gameDataReceived) {
            // Red inactive first → red hub is off during Shift 1
            // Blue inactive first → blue hub is off during Shift 1
            if (alliance.value() == frc::DriverStation::Alliance::kRed) {
              m_shift1Active = !redInactiveFirst;
            } else {
              m_shift1Active = redInactiveFirst;
            }
          }
        }
      }
    }

    m_matchTime = frc::DriverStation::GetMatchTime().value();
    if (m_matchTime < 0) {
      // Match time unavailable — default safe
      m_ourHubActive = true;
      m_shiftWarning = false;
      m_shiftRumble = false;
    } else {
      UpdateShiftState();
    }
  }

  // SmartDashboard telemetry
  frc::SmartDashboard::PutBoolean("Shift/Hub Active", m_ourHubActive);
  frc::SmartDashboard::PutBoolean("Shift/Warning", m_shiftWarning);
  frc::SmartDashboard::PutBoolean("Shift/Rumble", m_shiftRumble);
  frc::SmartDashboard::PutNumber("Shift/Current Shift", m_currentShift);
  frc::SmartDashboard::PutNumber("Shift/Match Time", m_matchTime);
  frc::SmartDashboard::PutNumber("Shift/Time Remaining in Shift",
                                 m_shiftTimeRemaining);
  frc::SmartDashboard::PutBoolean("Shift/Game Data Received",
                                  m_gameDataReceived);
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

void AllianceShiftSubsystem::UpdateShiftState() {
  using namespace ShiftConstants;

  // Determine shift window from match time and calculate countdown to next
  // shift. Match time counts DOWN (e.g. 135 → 0).
  if (m_matchTime > kShift1Start) {
    // Pre-shift-1 transition — both hubs active
    m_currentShift = 0;
    m_ourHubActive = true;
    m_shiftTimeRemaining = m_matchTime - kShift1Start;  // countdown to Shift 1
  } else if (m_matchTime > kShift2Start) {
    m_currentShift = 1;
    m_ourHubActive = m_shift1Active;
    m_shiftTimeRemaining = m_matchTime - kShift2Start;  // countdown to Shift 2
  } else if (m_matchTime > kShift3Start) {
    m_currentShift = 2;
    m_ourHubActive = !m_shift1Active;
    m_shiftTimeRemaining = m_matchTime - kShift3Start;  // countdown to Shift 3
  } else if (m_matchTime > kShift4Start) {
    m_currentShift = 3;
    m_ourHubActive = m_shift1Active;
    m_shiftTimeRemaining = m_matchTime - kShift4Start;  // countdown to Shift 4
  } else if (m_matchTime > kEndgameStart) {
    m_currentShift = 4;
    m_ourHubActive = !m_shift1Active;
    m_shiftTimeRemaining = m_matchTime - kEndgameStart;  // countdown to Endgame
  } else {
    // Endgame — both hubs active
    m_currentShift = 0;
    m_ourHubActive = true;
    m_shiftTimeRemaining = m_matchTime;  // time left in match
  }

  // No game data yet — safe default (hub always active)
  if (!m_gameDataReceived) {
    m_ourHubActive = true;
  }

  // Warning/rumble: fire within the warning window before each shift boundary
  auto inWindow = [&](double boundary, double window) {
    return (m_matchTime > boundary) && (m_matchTime <= boundary + window);
  };

  m_shiftWarning = inWindow(kShift1Start, kShiftWarningSeconds) ||
                   inWindow(kShift2Start, kShiftWarningSeconds) ||
                   inWindow(kShift3Start, kShiftWarningSeconds) ||
                   inWindow(kShift4Start, kShiftWarningSeconds) ||
                   inWindow(kEndgameStart, kShiftWarningSeconds);

  m_shiftRumble = inWindow(kShift1Start, kRumbleWarningSeconds) ||
                  inWindow(kShift2Start, kRumbleWarningSeconds) ||
                  inWindow(kShift3Start, kRumbleWarningSeconds) ||
                  inWindow(kShift4Start, kRumbleWarningSeconds) ||
                  inWindow(kEndgameStart, kRumbleWarningSeconds);
}
