// Copyright (c) 2025-2026 Rockin' Bots
// https://github.com/CarmelRobotics

package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.motorstatus.SimMotorStatus;

public class IntakeRollerIOSim implements IntakeRollerIO {
  private final DCMotorSim leftMotor;
  private final DCMotorSim rightMotor;

  private final SimMotorStatus leftStatus;
  private final SimMotorStatus rightStatus;

  public IntakeRollerIOSim() {
    this.leftMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            IntakeConstants.RollerConfigs.kSimMotor,
            IntakeConstants.RollerConfigs.kSimMotorMOIJ,
            IntakeConstants.RollerConfigs.kSimMotorGearRatio),
        IntakeConstants.RollerConfigs.kSimMotor);
    this.rightMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            IntakeConstants.RollerConfigs.kSimMotor,
            IntakeConstants.RollerConfigs.kSimMotorMOIJ,
            IntakeConstants.RollerConfigs.kSimMotorGearRatio),
        IntakeConstants.RollerConfigs.kSimMotor);

    this.leftStatus = new SimMotorStatus(leftMotor);
    this.rightStatus = new SimMotorStatus(rightMotor);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    leftMotor.setInputVoltage(voltage.in(Volts));
    rightMotor.setInputVoltage(-voltage.in(Volts));
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.leaderConnected = leftStatus.getConnected();
    inputs.leaderVelocity = leftStatus.getVelocity();
    inputs.leaderOutput = leftStatus.getOutput();
    inputs.leaderAppliedVoltage = leftStatus.getAppliedVoltage();
    inputs.leaderSupplyCurrent = leftStatus.getSupplyCurrent();
    inputs.leaderStatorCurrent = leftStatus.getStatorCurrent();
    inputs.leaderTemperature = leftStatus.getTemperature();

    inputs.followerConnected = rightStatus.getConnected();
    inputs.followerVelocity = rightStatus.getVelocity();
    inputs.followerOutput = rightStatus.getOutput();
    inputs.followerAppliedVoltage = rightStatus.getAppliedVoltage();
    inputs.followerSupplyCurrent = rightStatus.getSupplyCurrent();
    inputs.followerStatorCurrent = rightStatus.getStatorCurrent();
    inputs.followerTemperature = rightStatus.getTemperature();
  }
}
