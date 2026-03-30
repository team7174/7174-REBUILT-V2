// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/GenericHID.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>

#include <cmath>

RobotContainer::RobotContainer() {
  // Give the vision subsystem a reference to the drivetrain so it can
  // push MegaTag1 measurements into the pose estimator each loop.
  vision.SetDrivetrain(drivetrain);
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Shift rumble: vibrate both controllers for kRumbleWarningSeconds before
  // each alliance hub shift. Runs as the default command during teleop and
  // automatically clears rumble when teleop ends.
  allianceShift.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        double rumble = allianceShift.IsShiftRumble() ? 1.0 : 0.0;
        joystick.GetHID().SetRumble(frc::GenericHID::RumbleType::kBothRumble,
                                    rumble);
      },
      {&allianceShift}));

  // Clear rumble on any non-teleop mode (disabled, auto) so controllers
  // don't stay vibrating if teleop ends mid-rumble.
  frc2::RobotModeTriggers::Teleop().OnFalse(frc2::cmd::RunOnce([this] {
    joystick.GetHID().SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
  }));

  // Feed the aim subsystem the current robot pose every loop
  aim.SetDefaultCommand(frc2::cmd::Run(
      [this] { aim.UpdatePose(drivetrain.GetState().Pose); }, {&aim}));

  // Flywheel off by default — spins up only when right trigger is held.
  shooter.SetDefaultCommand(
      frc2::cmd::RunOnce([this] { shooter.SetRunning(false); }, {&shooter})
          .AndThen(frc2::cmd::Idle({&shooter})));

  // Intake rollers idle by default — sets idle once on boot then holds,
  // so toggle commands from the left trigger are not immediately overwritten.
  intake.SetDefaultCommand(frc2::cmd::RunOnce(
                               [this] {
                                 intake.SetRollerState(
                                     IntakeSubsystem::RollerState::kIdle);
                               },
                               {&intake})
                               .AndThen(frc2::cmd::Idle({&intake})));

  // Note that X is defined as forward according to WPILib convention,
  // and Y is defined as to the left according to WPILib convention.
  drivetrain.SetDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.ApplyRequest([this]() -> auto&& {
        return drive
            .WithVelocityX(-joystick.GetLeftY() *
                           MaxSpeed)  // Drive forward with negative Y (forward)
            .WithVelocityY(-joystick.GetLeftX() *
                           MaxSpeed)  // Drive left with negative X (left)
            .WithRotationalRate(-joystick.GetRightX() *
                                MaxAngularRate);  // Drive counterclockwise with
                                                  // negative X (left)
      }));

  // Idle while the robot is disabled. This ensures the configured
  // neutral mode is applied to the drive motors while disabled.
  frc2::RobotModeTriggers::Disabled().WhileTrue(
      drivetrain.ApplyRequest([] { return swerve::requests::Idle{}; })
          .IgnoringDisable(true));

  joystick.A()
      .WhileTrue(frc2::cmd::RunOnce(
                     [this] {
                       feeder.SetState(FeederSubsystem::FeederState::kFeeding);
                     },
                     {&feeder})
                     .AndThen(frc2::cmd::Idle({&feeder})))
      .OnFalse(frc2::cmd::RunOnce(
          [this] { feeder.SetState(FeederSubsystem::FeederState::kIdle); },
          {&feeder}));
  joystick.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
    return point.WithModuleDirection(
        frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
  }));

  // Right trigger held: turn to face hub (back of robot). Drive freely while
  // rotating; brake when sticks idle and aimed. Also agitates the intake
  // up and down to help move stuck balls. Spins up the shooter while held.
  joystick.RightTrigger(0.3)
      .WhileTrue(
          drivetrain
              .Run([this] {
                double lx = -joystick.GetLeftX();
                double ly = -joystick.GetLeftY();
                bool stickIdle = (std::abs(lx) < 0.1 && std::abs(ly) < 0.1);

                frc::Rotation2d fieldTarget{aim.robotHeading + 180_deg};
                frc::Rotation2d opPerspective =
                    drivetrain.GetOperatorForwardDirection();
                frc::Rotation2d adjTarget =
                    fieldTarget.RotateBy(-opPerspective);

                if (stickIdle && aim.IsAimed()) {
                  drivetrain.SetControl(brake);
                } else {
                  drivetrain.SetControl(aimDrive.WithVelocityX(ly * MaxSpeed)
                                            .WithVelocityY(lx * MaxSpeed)
                                            .WithTargetDirection(adjTarget));
                }
              })
              .AlongWith(frc2::cmd::Run(
                  [this] { shooter.SetFromDistance(aim.distance.value()); },
                  {&shooter}))
              .AlongWith(frc2::cmd::Run(
                  [this] {
                    // Start the agitate timer on first loop
                    if (!m_agitateTimer.IsRunning()) {
                      m_agitateTimer.Restart();
                    }

                    // Oscillate deploy angle every 0.4 s (0.8 s full cycle)
                    double elapsed = m_agitateTimer.Get().value();
                    bool isRaised = (static_cast<int>(elapsed / 0.4) % 2) == 1;

                    double targetAngleDeg = IntakeConstants::kDeployedAngleDeg +
                                            (isRaised ? 25.0 : 0.0);
                    intake.SetTargetAngleDeg(targetAngleDeg);
                  },
                  {&intake})))
      .OnFalse(frc2::cmd::RunOnce(
          [this] {
            shooter.SetRunning(false);
            shooter.SetHoodAngle(ShooterConstants::kHoodMinDegrees);
            m_agitateTimer.Stop();
            // Return intake to fully deployed (down) position
            intake.SetDeployState(IntakeSubsystem::DeployState::kDeployed);
          },
          {&shooter, &intake}));

  // Left trigger: toggle intake rollers on/off. Deploy stays down either way.
  // Uses WhileTrue so the command holds the subsystem while running,
  // preventing the default command from overriding it mid-press.
  joystick.LeftTrigger(0.3).OnTrue(
      frc2::cmd::RunOnce(
          [this] {
            m_intakeRollerOn = !m_intakeRollerOn;
            intake.SetDeployState(IntakeSubsystem::DeployState::kDeployed);
            intake.SetRollerState(m_intakeRollerOn
                                      ? IntakeSubsystem::RollerState::kIntaking
                                      : IntakeSubsystem::RollerState::kIdle);
          },
          {&intake})
          .AndThen(frc2::cmd::Idle({&intake})));

  // Left bumper: stop intake rollers only (deploy position unchanged).
  joystick.LeftBumper().OnTrue(frc2::cmd::RunOnce(
      [this] { intake.SetRollerState(IntakeSubsystem::RollerState::kIdle); },
      {&intake}));

  // Run SysId routines when holding back/start and X/Y.
  // Note that each routine should be run exactly once in a single log.
  (joystick.Back() && joystick.Y())
      .WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
  (joystick.Back() && joystick.X())
      .WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
  (joystick.Start() && joystick.Y())
      .WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  (joystick.Start() && joystick.X())
      .WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

  drivetrain.RegisterTelemetry(
      [this](auto const& state) { logger.Telemeterize(state); });
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // Simple drive forward auton
  return frc2::cmd::Sequence(
      // Reset our field centric heading to match the robot
      // facing away from our alliance station wall (0 deg).
      drivetrain.RunOnce(
          [this] { drivetrain.SeedFieldCentric(frc::Rotation2d{0_deg}); }),
      // Then slowly drive forward (away from us) for 5 seconds.
      drivetrain
          .ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(0.5_mps)
                .WithVelocityY(0_mps)
                .WithRotationalRate(0_tps);
          })
          .WithTimeout(5_s),
      // Finally idle for the rest of auton
      drivetrain.ApplyRequest([] { return swerve::requests::Idle{}; }));
}
