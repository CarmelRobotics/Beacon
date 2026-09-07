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
import frc.robot.subsystems.drive.module.ModuleIOSim;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(0);

  private final Drive drive = 
    new Drive(
      new GyroIO() {},
      new ModuleIOSim(DriveConstants.FrontLeft),
      new ModuleIOSim(DriveConstants.FrontRight),
      new ModuleIOSim(DriveConstants.BackLeft),
      new ModuleIOSim(DriveConstants.BackRight));

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
      drive.fieldCentricDrive(
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX(),
        0.1));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
