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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonSim;

public class RobotContainer {
  private final CommandXboxController controller;

  private final Drive drive;
  private final Vision vision;

  public RobotContainer() {
    this.controller = new CommandXboxController(0);

    switch (Constants.kCurrentMode) {
      case REAL -> {
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
      }
      case SIM -> {
        this.drive = new Drive(
          new GyroIO() {},
          new ModuleIOSim(DriveConstants.FrontLeft),
          new ModuleIOSim(DriveConstants.FrontRight),
          new ModuleIOSim(DriveConstants.BackLeft),
          new ModuleIOSim(DriveConstants.BackRight));

        this.vision = new Vision(
          drive::addVisionMeasurement,
          new VisionIO() {},
          new VisionIO() {});
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

    controller.povDown().onTrue(drive.seedFieldCentric());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
