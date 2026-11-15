// Copyright (c) 2025-2026 Rockin' Bots
// https://github.com/CarmelRobotics

package frc.robot.subsystems.shooter.roller;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.motorstatus.SimMotorStatus;

public class ShooterRollerIOSim implements ShooterRollerIO {
  private final DCMotorSim frontLeftMotor;
  private final DCMotorSim backLeftMotor;
  private final DCMotorSim frontRightMotor;
  private final DCMotorSim backRightMotor;

  private final SimMotorStatus frontLeftStatus;
  private final SimMotorStatus backLeftStatus;
  private final SimMotorStatus frontRightStatus;
  private final SimMotorStatus backRightStatus;

  public ShooterRollerIOSim() {
    this.frontLeftMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            ShooterConstants.RollerConfigs.kSimMotor,
            ShooterConstants.RollerConfigs.kSimMotorMOIJ,
            ShooterConstants.RollerConfigs.kSimMotorGearRatio),
        ShooterConstants.RollerConfigs.kSimMotor);

    this.backLeftMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            ShooterConstants.RollerConfigs.kSimMotor,
            ShooterConstants.RollerConfigs.kSimMotorMOIJ,
            ShooterConstants.RollerConfigs.kSimMotorGearRatio),
        ShooterConstants.RollerConfigs.kSimMotor);

    this.frontRightMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            ShooterConstants.RollerConfigs.kSimMotor,
            ShooterConstants.RollerConfigs.kSimMotorMOIJ,
            ShooterConstants.RollerConfigs.kSimMotorGearRatio),
        ShooterConstants.RollerConfigs.kSimMotor);

    this.backRightMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            ShooterConstants.RollerConfigs.kSimMotor,
            ShooterConstants.RollerConfigs.kSimMotorMOIJ,
            ShooterConstants.RollerConfigs.kSimMotorGearRatio),
        ShooterConstants.RollerConfigs.kSimMotor);

    this.frontLeftStatus = new SimMotorStatus(frontLeftMotor);
    this.backLeftStatus = new SimMotorStatus(backLeftMotor);
    this.frontRightStatus = new SimMotorStatus(frontRightMotor);
    this.backRightStatus = new SimMotorStatus(backRightMotor);
  }

  @Override
  public void updatePID(double kA, double kD, double kI, double kP, double kS, double kV) {
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    frontLeftMotor.setAngularVelocity(velocity.in(RadiansPerSecond));
    backLeftMotor.setAngularVelocity(velocity.in(RadiansPerSecond));
    frontRightMotor.setAngularVelocity(-(velocity.in(RadiansPerSecond)));
    backRightMotor.setAngularVelocity(-(velocity.in(RadiansPerSecond)));
  }

  @Override
  public void setOutput(double output) {
    frontLeftMotor.setInputVoltage(12.0 * output);
    backLeftMotor.setInputVoltage(12.0 * output);
    frontRightMotor.setInputVoltage(-12.0 * output);
    backRightMotor.setInputVoltage(-12.0 * output);
  }

  @Override
  public void updateInputs(ShooterRollerIOInputsAutoLogged inputs) {
    inputs.frontLeftConnected = frontLeftStatus.getConnected();
    inputs.frontLeftVelocity = frontLeftStatus.getVelocity();
    inputs.frontLeftOutput = frontLeftStatus.getOutput();
    inputs.frontLeftAppliedVoltage = frontLeftStatus.getAppliedVoltage();
    inputs.frontLeftSupplyCurrent = frontLeftStatus.getSupplyCurrent();
    inputs.frontLeftStatorCurrent = frontLeftStatus.getStatorCurrent();
    inputs.frontLeftTemperature = frontLeftStatus.getTemperature();

    inputs.backLeftConnected = backLeftStatus.getConnected();
    inputs.backLeftVelocity = backLeftStatus.getVelocity();
    inputs.backLeftOutput = backLeftStatus.getOutput();
    inputs.backLeftAppliedVoltage = backLeftStatus.getAppliedVoltage();
    inputs.backLeftSupplyCurrent = backLeftStatus.getSupplyCurrent();
    inputs.backLeftStatorCurrent = backLeftStatus.getStatorCurrent();
    inputs.backLeftTemperature = backLeftStatus.getTemperature();

    inputs.frontRightConnected = frontRightStatus.getConnected();
    inputs.frontRightVelocity = frontRightStatus.getVelocity();
    inputs.frontRightOutput = frontRightStatus.getOutput();
    inputs.frontRightAppliedVoltage = frontRightStatus.getAppliedVoltage();
    inputs.frontRightSupplyCurrent = frontRightStatus.getSupplyCurrent();
    inputs.frontRightStatorCurrent = frontRightStatus.getStatorCurrent();
    inputs.frontRightTemperature = frontRightStatus.getTemperature();

    inputs.backRightConnected = backRightStatus.getConnected();
    inputs.backRightVelocity = backRightStatus.getVelocity();
    inputs.backRightOutput = backRightStatus.getOutput();
    inputs.backRightAppliedVoltage = backRightStatus.getAppliedVoltage();
    inputs.backRightSupplyCurrent = backRightStatus.getSupplyCurrent();
    inputs.backRightStatorCurrent = backRightStatus.getStatorCurrent();
    inputs.backRightTemperature = backRightStatus.getTemperature();
  }
}
