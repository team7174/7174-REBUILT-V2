// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc2/command/CommandScheduler.h>

Robot::Robot() {
  // Start WPILib data logging. Logs are written to /home/lvuser/logs/ on the
  // roboRIO. Retrieve them with: VS Code → WPILib: Download Logs, or via SFTP.
  frc::DataLogManager::Start();

  // Mirror console output into the log so warnings appear in the .wpilog too.
  frc::DataLogManager::LogConsoleOutput(true);

  // Also log DS control/joystick data so it appears in the same log.
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
}

void Robot::RobotPeriodic() {
  m_timeAndJoystickReplay.Update();
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand.value());
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    frc2::CommandScheduler::GetInstance().Cancel(m_autonomousCommand.value());
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() { frc2::CommandScheduler::GetInstance().CancelAll(); }

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
