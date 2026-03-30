// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/GenericHID.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include <cmath>

RobotContainer::RobotContainer() {
  // Give the vision subsystem a reference to the drivetrain so it can
  // push MegaTag1 measurements into the pose estimator each loop.
  vision.SetDrivetrain(drivetrain);

  // IZone: only accumulate I-gain when heading error is within 5° (0.087 rad).
  // Prevents windup during large turns but still kills steady-state error.
  aimDrive.HeadingController.SetIZone(0.087);

  // ── PathPlanner named commands ───────────────────────────────────────────
  // Must be registered BEFORE ConfigureAutoBuilder() / buildAutoChooser().

  // "Intake On" — turns on intake rollers for the duration of the path event.
  pathplanner::NamedCommands::registerCommand(
      "Intake",
      frc2::cmd::RunOnce([this] { m_intakeOn = true; })
          .AndThen(frc2::cmd::RunOnce(
              [this] {
                intake.SetRollerState(IntakeSubsystem::RollerState::kIntaking);
              },
              {&intake})));

  // "Auto Shoot" — turns to face the hub (back of robot), spins up the
  // shooter, feeds once flywheel + hood are stable and aimed, then stops.
  // Times out after 4 seconds so the auto path keeps moving if the shot stalls.
  pathplanner::NamedCommands::registerCommand(
      "AutoShoot",
      // 1. Turn to face hub — keep translating at zero while locking heading
      drivetrain
          .Run([this] {
            frc::Rotation2d fieldTarget{aim.robotHeading + 180_deg};
            frc::Rotation2d opPerspective =
                drivetrain.GetOperatorForwardDirection();
            frc::Rotation2d adjTarget = fieldTarget.RotateBy(-opPerspective);
            drivetrain.SetControl(aimDrive.WithVelocityX(0_mps)
                                      .WithVelocityY(0_mps)
                                      .WithTargetDirection(adjTarget));
          })
          // 2. Spin up flywheel + set hood from live distance
          .AlongWith(frc2::cmd::Run(
              [this] { shooter.SetFromDistance(aim.distance.value()); },
              {&shooter}))
          // 3. Feed only when aimed, at speed, and hood is ready
          .AlongWith(frc2::cmd::Run(
              [this] {
                bool aimed = aim.IsAimed();
                bool atSpeed = shooter.IsStableAtSpeed();
                bool hoodReady = shooter.IsHoodAtTarget();
                frc::SmartDashboard::PutBoolean("Auto/Aimed", aimed);
                frc::SmartDashboard::PutBoolean("Auto/AtSpeed", atSpeed);
                frc::SmartDashboard::PutBoolean("Auto/HoodReady", hoodReady);
                if (aimed && atSpeed && hoodReady) {
                  feeder.SetState(FeederSubsystem::FeederState::kFeeding);
                } else {
                  feeder.SetState(FeederSubsystem::FeederState::kIdle);
                }
              },
              {&feeder}))
          .WithTimeout(4_s)
          .AndThen(frc2::cmd::RunOnce(
              [this] {
                shooter.SetRunning(false);
                shooter.SetHoodAngle(ShooterConstants::kHoodMinDegrees);
                feeder.SetState(FeederSubsystem::FeederState::kIdle);
              },
              {&shooter, &feeder})));

  // Configure PathPlanner AutoBuilder with this robot's swerve drivetrain.
  // Must happen before buildAutoChooser() or any PathPlannerAuto is created.
  drivetrain.ConfigureAutoBuilder();

  // Populate the chooser with every auto found in deploy/pathplanner/autos/.
  // Shows up on SmartDashboard / Shuffleboard under "Auto Chooser".
  m_autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
  frc::SmartDashboard::PutData("Auto Chooser", &m_autoChooser);

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

  // Reset shooter/feeder when teleop starts so auto-shoot state doesn't
  // carry over from autonomous.
  frc2::RobotModeTriggers::Teleop().OnTrue(frc2::cmd::RunOnce(
      [this] {
        shooter.SetRunning(false);
        shooter.SetHoodAngle(ShooterConstants::kHoodMinDegrees);
        feeder.SetState(FeederSubsystem::FeederState::kIdle);
        m_intakeOn = true;
      },
      {&shooter, &feeder}));

  // Feed the aim subsystem the current robot pose every loop
  aim.SetDefaultCommand(frc2::cmd::Run(
      [this] { aim.UpdatePose(drivetrain.GetState().Pose); }, {&aim}));

  // Flywheel idles at low RPM by default — keeps it warm for fast spin-up.
  // Right trigger overrides with full shooting speed from interpolation map.
  shooter.SetDefaultCommand(
      frc2::cmd::RunOnce([this] { shooter.SetRunning(false); }, {&shooter})
          .AndThen(frc2::cmd::Idle({&shooter})));

  // Intake default command — deploy chain broken, so intake stays down always.
  // Rollers run based on m_intakeOn flag (controlled by left trigger / bumper).
  intake.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        // Deploy motor disabled (chain broken) — do not command position
        if (m_intakeOn) {
          intake.SetRollerState(IntakeSubsystem::RollerState::kIntaking);
        } else {
          intake.SetRollerState(IntakeSubsystem::RollerState::kIdle);
        }
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

  // A button: manual feed override — forces feeder on regardless of flywheel
  // speed. Works even while right trigger is held (the RT feeder lambda checks
  // m_manualFeedOverride). Also works standalone when RT is not held.
  joystick.A()
      .WhileTrue(frc2::cmd::RunOnce([this] {
                   m_manualFeedOverride = true;
                   feeder.SetState(FeederSubsystem::FeederState::kFeeding);
                 }).AndThen(frc2::cmd::Idle()))
      .OnFalse(frc2::cmd::RunOnce([this] {
        m_manualFeedOverride = false;
        feeder.SetState(FeederSubsystem::FeederState::kIdle);
      }));
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
                    // OR: A-button manual override forces feeding immediately.
                    bool aimed = aim.IsAimed();
                    bool atSpeed = shooter.IsStableAtSpeed();
                    bool hoodReady = shooter.IsHoodAtTarget();
                    bool manualOverride = m_manualFeedOverride;
                    frc::SmartDashboard::PutBoolean("Feed/Aimed", aimed);
                    frc::SmartDashboard::PutBoolean("Feed/AtSpeed", atSpeed);
                    frc::SmartDashboard::PutBoolean("Feed/HoodReady",
                                                    hoodReady);
                    frc::SmartDashboard::PutBoolean("Feed/ManualOverride",
                                                    manualOverride);
                    frc::SmartDashboard::PutBoolean(
                        "Feed/ReadyToFire",
                        manualOverride || (aimed && atSpeed && hoodReady));
                    if (manualOverride || (aimed && atSpeed && hoodReady)) {
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
            m_manualFeedOverride = false;
            m_agitateTimer.Stop();
            // Resume intaking — rollers back on (deploy chain broken, stays
            // down)
            m_intakeOn = true;
            intake.SetRollerState(IntakeSubsystem::RollerState::kIntaking);
          },
          {&shooter, &intake, &feeder}));

  // Left trigger: enable intake rollers (deploy chain broken — no deploy
  // command).
  joystick.LeftTrigger(0.3).OnTrue(
      frc2::cmd::RunOnce([this] { m_intakeOn = true; }));

  // Left bumper: stop rollers (deploy chain broken — no stow command).
  joystick.LeftBumper().OnTrue(
      frc2::cmd::RunOnce([this] { m_intakeOn = false; }));

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

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autoChooser.GetSelected();
}
