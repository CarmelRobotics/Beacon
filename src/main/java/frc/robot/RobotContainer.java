// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.pivot.IntakePivotIO;
import frc.robot.subsystems.intake.pivot.IntakePivotIOSim;
import frc.robot.subsystems.intake.pivot.IntakePivotIOTalonFX;
import frc.robot.subsystems.intake.roller.IntakeRollerIO;
import frc.robot.subsystems.intake.roller.IntakeRollerIOSim;
import frc.robot.subsystems.intake.roller.IntakeRollerIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.geometry.FieldConstants;

public class RobotContainer {
  private final CommandXboxController controller;

  private final Drive drive;
  @SuppressWarnings("unused")
  private final Vision vision;

  private final Indexer indexer;
  private final Intake intake;

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
            new VisionIOLimelight(VisionConstants.kLimelightRight.name(), drive::getRotation));
        
        this.indexer = new Indexer(new IndexerIOTalonFX());
        this.intake = new Intake(new IntakePivotIOTalonFX(), new IntakeRollerIOTalonFX());
      }
      case kSim -> {
        this.drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(DriveConstants.FrontLeft),
            new ModuleIOSim(DriveConstants.FrontRight),
            new ModuleIOSim(DriveConstants.BackLeft),
            new ModuleIOSim(DriveConstants.BackRight));

        this.vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIO() {},
            new VisionIO() {});

        this.indexer = new Indexer(new IndexerIOSim());
        this.intake = new Intake(new IntakePivotIOSim(), new IntakeRollerIOSim());
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

        this.indexer = new Indexer(new IndexerIO() {});
        this.intake = new Intake(new IntakePivotIO() {}, new IntakeRollerIO() {});
      }
    }

    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        drive.fieldCentricDrive(
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            0.1));

    controller.leftTrigger().whileTrue(
      drive.lockedDrive(
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        0.1,
        () -> FieldConstants.Hub.innerCenterPoint.toTranslation2d()));

    controller.povDown().onTrue(drive.seedFieldCentric());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
