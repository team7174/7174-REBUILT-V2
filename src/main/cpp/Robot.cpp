// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc2/command/CommandScheduler.h>

Robot::Robot() {}

void Robot::RobotPeriodic() {
  m_timeAndJoystickReplay.Update();
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {
  // Switch to coast so operators can move the hood and intake by hand
  m_container.intake.SetDeployBrakeMode(false);
  m_container.shooter.SetHoodBrakeMode(false);
}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {
  // Re-enable brake mode before any match phase starts
  m_container.intake.SetDeployBrakeMode(true);
  m_container.shooter.SetHoodBrakeMode(true);
}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand);
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    frc2::CommandScheduler::GetInstance().Cancel(m_autonomousCommand);
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
