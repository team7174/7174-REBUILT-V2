// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include "Telemetry.h"
#include "subsystems/AimSubsystem.h"
#include "subsystems/AllianceShiftSubsystem.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/FeederSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/VisionSubsystem.h"

class RobotContainer {
 private:
  units::meters_per_second_t MaxSpeed =
      1.0 *
      TunerConstants::kSpeedAt12Volts;  // kSpeedAt12Volts desired top speed
  units::radians_per_second_t MaxAngularRate =
      0.75_tps;  // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  swerve::requests::FieldCentric drive =
      swerve::requests::FieldCentric{}
          .WithDeadband(MaxSpeed * 0.1)
          .WithRotationalDeadband(MaxAngularRate * 0.1)  // Add a 10% deadband
          .WithDriveRequestType(
              swerve::DriveRequestType::
                  OpenLoopVoltage);  // Use open-loop control for drive motors
  swerve::requests::SwerveDriveBrake brake{};
  swerve::requests::PointWheelsAt point{};

  // Heading-locked drive for aiming: translates freely while keeping the
  // robot heading locked to aim.robotHeading (back of robot faces the hub).
  swerve::requests::FieldCentricFacingAngle aimDrive =
      swerve::requests::FieldCentricFacingAngle{}
          .WithDeadband(MaxSpeed * 0.1)
          .WithRotationalDeadband(MaxAngularRate * 0.02)
          .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage)
          .WithHeadingPID(
              7.0, 0.3,
              0.5);  // I-gain with IZone eliminates steady-state error

  /* Note: This must be constructed before the drivetrain, otherwise we need to
   *       define a destructor to un-register the telemetry from the drivetrain
   */
  Telemetry logger{MaxSpeed};

  frc2::CommandXboxController joystick{0};

  // Timer used to oscillate the intake deploy angle when agitating
  frc::Timer m_agitateTimer;

  // Whether intake is enabled (left trigger on, left bumper off)
  bool m_intakeOn = false;

  // Auto chooser populated by PathPlanner — shown on
  // SmartDashboard/Shuffleboard
  frc::SendableChooser<frc2::Command*> m_autoChooser;

 public:
  subsystems::CommandSwerveDrivetrain drivetrain{
      TunerConstants::CreateDrivetrain()};
  AimSubsystem aim;
  AllianceShiftSubsystem allianceShift;
  FeederSubsystem feeder;
  IntakeSubsystem intake;
  ShooterSubsystem shooter;
  VisionSubsystem vision;

  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  void ConfigureBindings();
};
