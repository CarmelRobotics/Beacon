// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.FieldRelative;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakePivotState;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRollerState;
import frc.robot.subsystems.intake.pivot.IntakePivotIO;
import frc.robot.subsystems.intake.pivot.IntakePivotIOSim;
import frc.robot.subsystems.intake.pivot.IntakePivotIOTalonFX;
import frc.robot.subsystems.intake.roller.IntakeRollerIO;
import frc.robot.subsystems.intake.roller.IntakeRollerIOSim;
import frc.robot.subsystems.intake.roller.IntakeRollerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterKickerState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterPivotState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterRollerState;
import frc.robot.subsystems.shooter.kicker.ShooterKickerIO;
import frc.robot.subsystems.shooter.kicker.ShooterKickerIOSim;
import frc.robot.subsystems.shooter.kicker.ShooterKickerIOTalonFX;
import frc.robot.subsystems.shooter.pivot.ShooterPivotIO;
import frc.robot.subsystems.shooter.pivot.ShooterPivotIOSim;
import frc.robot.subsystems.shooter.pivot.ShooterPivotIOTalonFX;
import frc.robot.subsystems.shooter.roller.ShooterRollerIO;
import frc.robot.subsystems.shooter.roller.ShooterRollerIOSim;
import frc.robot.subsystems.shooter.roller.ShooterRollerIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;

public class RobotContainer {
  private final CommandXboxController controller;

  private final Drive drive;
  @SuppressWarnings("unused")
  private final Vision vision;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;
  private final FieldRelative field;
  private final Auto auto;

  public RobotContainer() {
    this.controller = new CommandXboxController(0);

    switch (Constants.kCurrentMode) {
      case kReal -> {
        this.drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(DriveConstants.FrontLeft),
            new ModuleIOTalonFX(DriveConstants.FrontRight),
            new ModuleIOTalonFX(DriveConstants.BackLeft),
            new ModuleIOTalonFX(DriveConstants.BackRight));
        this.vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOLimelight(VisionConstants.kLimelightLeft.name(), drive::getRotation),
            new VisionIOLimelight(VisionConstants.kLimelightRight.name(), drive::getRotation)
            );
        
        this.shooter = new Shooter(
          new ShooterKickerIOTalonFX(),
          new ShooterPivotIOTalonFX(),
          new ShooterRollerIOTalonFX(),
          drive::getPose,
          drive::isFacingTarget);
        this.indexer = new Indexer(new IndexerIOTalonFX(), shooter::isReadyToFeed);
        this.intake = new Intake(new IntakePivotIOTalonFX(), new IntakeRollerIOTalonFX(), shooter::isReadyToFeed);
      }
      case kSim -> {
        this.drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(DriveConstants.FrontLeft),
            new ModuleIOSim(DriveConstants.FrontRight),
            new ModuleIOSim(DriveConstants.BackLeft),
            new ModuleIOSim(DriveConstants.BackRight));
        /* Vision disabled in simulator for performance. */
        this.vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIO() {},
            new VisionIO() {});
        this.shooter = new Shooter(
          new ShooterKickerIOSim(),
          new ShooterPivotIOSim(),
          new ShooterRollerIOSim(),
          drive::getPose,
          drive::isFacingTarget
        );
        this.indexer = new Indexer(new IndexerIOSim(), shooter::isReadyToFeed);
        this.intake = new Intake(new IntakePivotIOSim(), new IntakeRollerIOSim(), shooter::isReadyToFeed);
      }
      default -> {
        this.drive = new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
        this.vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIO() {},
            new VisionIO() {});
        this.shooter = new Shooter(
          new ShooterKickerIO() {},
          new ShooterPivotIO() {},
          new ShooterRollerIO() {},
          drive::getPose,
          drive::isFacingTarget);
        this.indexer = new Indexer(new IndexerIO() {}, shooter::isReadyToFeed);
        this.intake = new Intake(new IntakePivotIO() {}, new IntakeRollerIO() {}, shooter::isReadyToFeed);
      }
    }

    this.field = new FieldRelative(drive::getPose);
    this.auto = new Auto(drive, shooter, indexer, intake, field);
    configureBindings();
  }

  private void configureBindings() {
    /*
     * Left Joystick: Swerve Translation
     * Right Joystick: Swerve Rotation
     * POV Down: Seed Field Centric
     * 
     * Left Trigger: Deploy intake, while held, run intake rollers.
     * Right Trigger: Shoot fuel.
     * POV Up: Toggle Deploy/Stow intake.
     */

    final Command controllerWarn = Commands.runEnd(
      () -> controller.setRumble(RumbleType.kBothRumble, 1.0),
      () -> controller.setRumble(RumbleType.kBothRumble, 0));

    final Command intakeDeploy = intake.setState(IntakePivotState.kDeploy);
    final Command intakeIdle = intake.setState(IntakeRollerState.kIdle);
    final Command intakeRun = intake.setState(IntakeRollerState.kIntake);
    final Command intakeAgitate = intake.setState(IntakePivotState.kAgitate, IntakeRollerState.kIntake);
    final Command intakeIdleAndDeploy = intake.setState(IntakePivotState.kDeploy, IntakeRollerState.kIdle);
    final Command intakeToggle =
      intake.setState(
        () -> intake.getPivotState() == IntakePivotState.kStow 
          ? IntakePivotState.kDeploy
          : IntakePivotState.kStow);

    final Command shooterIdle = shooter.setState(ShooterKickerState.kIdle, ShooterPivotState.kStow, ShooterRollerState.kIdle);
    final Command shooterScore = shooter.setState(ShooterKickerState.kFeed, ShooterPivotState.kScore, ShooterRollerState.kScore);
    final Command shooterPass = shooter.setState(ShooterKickerState.kFeed, ShooterPivotState.kPass, ShooterRollerState.kPass);

    final Command indexerIdle = indexer.setState(IndexerState.kIdle);
    final Command indexerFeed = indexer.setState(IndexerState.kFeed);
    
    /* Drivetrain Controls */
    drive.setDefaultCommand(
        drive.fieldCentricDrive(
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            0.1));
    controller.povDown().onTrue(drive.seedFieldCentric());

    /* Intake */
    controller.leftTrigger().onTrue(intakeDeploy);
    controller.leftTrigger()
      .onTrue(intakeRun)
      .onFalse(intakeIdle);
    controller.povUp().onTrue(intakeToggle);

    /* Shooter Scoring */
    controller.rightTrigger().and(field::isInAllianceZone).whileTrue(
        drive.targetedDrive(
          () -> -controller.getLeftY(),
          () -> -controller.getLeftX(),
          0.1,
          field::hubTarget));
    controller.rightTrigger().and(field::isNotInAllianceZone).whileTrue(controllerWarn);
    controller.rightTrigger().and(field::isInAllianceZone)
        .onTrue(shooterScore)
        .onFalse(shooterIdle);
    controller.rightTrigger().and(field::isInAllianceZone)
        .onTrue(indexerFeed)
        .onFalse(indexerIdle);
    controller.rightTrigger().and(field::isInAllianceZone)
        .whileTrue(intakeAgitate)
        .onFalse(intakeIdleAndDeploy);

    /* Shooter Passing */
    controller.rightBumper().and(field::isPassable).whileTrue(
        drive.targetedDrive(
          () -> -controller.getLeftY(),
          () -> -controller.getLeftX(),
          0.1,
          field::passingTarget));
    controller.rightBumper().and(field::isPassable)
        .onTrue(shooterPass)
        .onFalse(shooterIdle);
    controller.rightBumper().and(field::isPassable)
        .whileTrue(indexerFeed)
        .onFalse(indexerIdle);
    controller.rightBumper().and(field::isPassable)
        .whileTrue(intakeAgitate)
        .onFalse(intakeIdleAndDeploy);
    controller.rightBumper().and(field::isNotPassable)
        .whileTrue(controllerWarn);
  }

  public Command getTeleoperatedCommand() {
    /* Set everything to idle after auto */
    return Commands.parallel(
        drive.runOnce(drive::stop),
        indexer.setState(IndexerState.kIdle),
        intake.setState(IntakeRollerState.kIdle),
        shooter.setState(ShooterKickerState.kIdle, ShooterPivotState.kStow, ShooterRollerState.kIdle)
    );
  }

  public Command getAutonomousCommand() {
    return auto.getSelected();
  }
}
