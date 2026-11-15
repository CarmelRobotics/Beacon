// Copyright (c) 2025-2026 Rockin' Bots
// https://github.com/CarmelRobotics

package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.motorstatus.SimMotorStatus;

public class IntakePivotIOSim implements IntakePivotIO {
  private final DCMotorSim leftMotor;
  private final DCMotorSim rightMotor;

  private final SimMotorStatus leftStatus;
  private final SimMotorStatus rightStatus;

  public IntakePivotIOSim() {
    this.leftMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            IntakeConstants.PivotConfigs.kSimMotor,
            IntakeConstants.PivotConfigs.kSimMotorMOIJ,
            IntakeConstants.PivotConfigs.kSimMotorGearRatio),
        IntakeConstants.PivotConfigs.kSimMotor);
    this.rightMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            IntakeConstants.PivotConfigs.kSimMotor,
            IntakeConstants.PivotConfigs.kSimMotorMOIJ,
            IntakeConstants.PivotConfigs.kSimMotorGearRatio),
        IntakeConstants.PivotConfigs.kSimMotor);
    this.leftStatus = new SimMotorStatus(leftMotor);
    this.rightStatus = new SimMotorStatus(rightMotor);
  }

  @Override
  public void zeroHere() {}

  @Override
  public void updatePID(double kA, double kD, double kI, double kP, double kS, double kV) {}

  @Override
  public void setPosition(Angle position) {
    leftMotor.setAngle(position.in(Radians));
    rightMotor.setAngle(-position.in(Radians));
  }

  @Override
  public void updateInputs(IntakePivotIOInputsAutoLogged inputs) {
    inputs.leaderConnected = leftStatus.getConnected();
    inputs.leaderPosition = leftStatus.getPosition();
    inputs.leaderVelocity = leftStatus.getVelocity();
    inputs.leaderOutput = leftStatus.getOutput();
    inputs.leaderAppliedVoltage = leftStatus.getAppliedVoltage();
    inputs.leaderStatorCurrent = leftStatus.getSupplyCurrent();
    inputs.leaderTemperature = leftStatus.getTemperature();

    inputs.followerConnected = rightStatus.getConnected();
    inputs.followerPosition = rightStatus.getPosition();
    inputs.followerVelocity = rightStatus.getVelocity();
    inputs.leaderOutput = rightStatus.getOutput();
    inputs.followerAppliedVoltage = rightStatus.getAppliedVoltage();
    inputs.followerStatorCurrent = rightStatus.getSupplyCurrent();
    inputs.followerTemperature = rightStatus.getTemperature();
  }
}
