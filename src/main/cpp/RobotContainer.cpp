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

  // IZone: only accumulate I-gain when heading error is within 5° (0.087 rad).
  // Prevents windup during large turns but still kills steady-state error.
  aimDrive.HeadingController.SetIZone(0.087);

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

  // Flywheel idles at low RPM by default — keeps it warm for fast spin-up.
  // Right trigger overrides with full shooting speed from interpolation map.
  shooter.SetDefaultCommand(
      frc2::cmd::RunOnce([this] { shooter.SetRunning(false); }, {&shooter})
          .AndThen(frc2::cmd::Idle({&shooter})));

  // Intake default command — deploy chain broken, so intake stays down always.
  // Rollers run continuously regardless of m_intakeOn.
  intake.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        // Deploy motor disabled (chain broken) — do not command position
        intake.SetRollerState(IntakeSubsystem::RollerState::kIntaking);
      },
      {&intake}));

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

  // A button: manual feed backup — runs feeder directly without taking
  // subsystem ownership, so it cannot preempt or interrupt the right trigger
  // shooting command group.
  joystick.A()
      .WhileTrue(frc2::cmd::RunOnce([this] {
                   feeder.SetState(FeederSubsystem::FeederState::kFeeding);
                 }).AndThen(frc2::cmd::Idle()))
      .OnFalse(frc2::cmd::RunOnce(
          [this] { feeder.SetState(FeederSubsystem::FeederState::kIdle); }));
  joystick.B().WhileTrue(
      drivetrain
          .ApplyRequest([this]() -> auto&& {
            return point.WithModuleDirection(
                frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
          })
          .AlongWith(frc2::cmd::Run(
              [this] {
                intake.SetRollerState(IntakeSubsystem::RollerState::kIdle);
              },
              {&intake})));

  // Right trigger held: turn to face hub (back of robot). Drive freely while
  // rotating; brake when sticks idle and aimed. Also agitates the intake
  // up and down to help move stuck balls. Spins up the shooter while held.
  // Rollers are OFF during shooting to avoid feeding extra balls.
  // Feeder auto-engages once the flywheel is stable at speed.
  joystick.RightTrigger(0.3)
      .WhileTrue(
          drivetrain
              .Run([this] {
                double lx = -joystick.GetLeftX();
                double ly = -joystick.GetLeftY();

                frc::Rotation2d fieldTarget{aim.robotHeading + 180_deg};
                frc::Rotation2d opPerspective =
                    drivetrain.GetOperatorForwardDirection();
                frc::Rotation2d adjTarget =
                    fieldTarget.RotateBy(-opPerspective);

                drivetrain.SetControl(aimDrive.WithVelocityX(ly * MaxSpeed)
                                          .WithVelocityY(lx * MaxSpeed)
                                          .WithTargetDirection(adjTarget));
              })
              .AlongWith(frc2::cmd::Run(
                  [this] {
                    // Use interpolation map for hub shots, hardcoded for passes
                    if (aim.IsPassingMode()) {
                      shooter.SetHoodAngle(45.0);
                      shooter.SetFlywheelRPM(2500.0);
                    } else {
                      shooter.SetFromDistance(aim.distance.value());
                    }
                  },
                  {&shooter}))
              .AlongWith(frc2::cmd::Run(
                  [this] {
                    // Rollers OFF while shooting
                    intake.SetRollerState(IntakeSubsystem::RollerState::kIdle);

                    // Start the agitate timer on first loop
                    if (!m_agitateTimer.IsRunning()) {
                      m_agitateTimer.Restart();
                    }

                    // Oscillate deploy angle every 0.267 s (0.533 s full cycle)
                    double elapsed = m_agitateTimer.Get().value();
                    bool isRaised =
                        (static_cast<int>(elapsed / 0.267) % 2) == 1;

                    double targetAngleDeg = IntakeConstants::kDeployedAngleDeg +
                                            (isRaised ? 110.0 : 0.0);
                    intake.SetTargetAngleDeg(targetAngleDeg);
                  },
                  {&intake}))
              .AlongWith(frc2::cmd::Run(
                  [this] {
                    // All three conditions must be met before feeding:
                    //   1. Robot is aimed at target (within heading tolerance)
                    //   2. Flywheel is stable at the correct speed
                    //   3. Hood is at its target angle
                    bool aimed = aim.IsAimed();
                    bool atSpeed = shooter.IsStableAtSpeed();
                    bool hoodReady = shooter.IsHoodAtTarget();
                    frc::SmartDashboard::PutBoolean("Feed/Aimed", aimed);
                    frc::SmartDashboard::PutBoolean("Feed/AtSpeed", atSpeed);
                    frc::SmartDashboard::PutBoolean("Feed/HoodReady",
                                                    hoodReady);
                    frc::SmartDashboard::PutBoolean(
                        "Feed/ReadyToFire", aimed && atSpeed && hoodReady);
                    if (aimed && atSpeed && hoodReady) {
                      feeder.SetState(FeederSubsystem::FeederState::kFeeding);
                    } else {
                      feeder.SetState(FeederSubsystem::FeederState::kIdle);
                    }
                  },
                  {&feeder})))
      .OnFalse(frc2::cmd::RunOnce(
          [this] {
            shooter.SetRunning(false);
            shooter.SetHoodAngle(ShooterConstants::kHoodMinDegrees);
            feeder.SetState(FeederSubsystem::FeederState::kIdle);
            m_agitateTimer.Stop();
            // Resume intaking — rollers back on (deploy chain broken, stays
            // down)
            intake.SetRollerState(IntakeSubsystem::RollerState::kIntaking);
          },
          {&shooter, &intake, &feeder}));

  // Left trigger: rollers on (deploy chain broken — no deploy command).
  joystick.LeftTrigger(0.3).OnTrue(frc2::cmd::RunOnce(
      [this] {
        intake.SetRollerState(IntakeSubsystem::RollerState::kIntaking);
      },
      {&intake}));

  // Left bumper: stop rollers (deploy chain broken — no stow command).
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
