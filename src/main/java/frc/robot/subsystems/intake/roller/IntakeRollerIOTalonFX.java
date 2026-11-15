// Copyright (c) 2025-2026 Rockin' Bots
// https://github.com/CarmelRobotics

package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.motorstatus.TalonFXStatus;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private final TalonFX leftLeaderMotor;
  private final TalonFX rightFollowerMotor;

  private final TalonFXStatus leaderStatus;
  private final TalonFXStatus followerStatus;

  private final VoltageOut voltageRequest;

  public IntakeRollerIOTalonFX() {
    this.leftLeaderMotor = new TalonFX(IntakeConstants.kLeftLeaderRollerMotorId);
    this.rightFollowerMotor = new TalonFX(IntakeConstants.kRightFollowerRollerMotorId);

    this.leaderStatus = new TalonFXStatus(leftLeaderMotor);
    this.followerStatus = new TalonFXStatus(rightFollowerMotor);

    this.voltageRequest = new VoltageOut(0);

    PhoenixUtil.tryUntilOk(Constants.kMaxAttempts,
        () -> leftLeaderMotor.getConfigurator().apply(IntakeConstants.RollerConfigs.TalonFXConfiguration,
            Constants.kTimeoutSeconds));
    PhoenixUtil.tryUntilOk(Constants.kMaxAttempts,
        () -> rightFollowerMotor.getConfigurator().apply(IntakeConstants.RollerConfigs.TalonFXConfiguration,
            Constants.kTimeoutSeconds));

    rightFollowerMotor.setControl(new Follower(leftLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    leaderStatus.setUpdateFrequency(IntakeConstants.kStatusFrequency);
    followerStatus.setUpdateFrequency(IntakeConstants.kStatusFrequency);
    ParentDevice.optimizeBusUtilizationForAll(leftLeaderMotor, rightFollowerMotor);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    leftLeaderMotor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    leaderStatus.refresh();
    followerStatus.refresh();

    inputs.leaderConnected = leaderStatus.getConnected();
    inputs.leaderVelocity = leaderStatus.getVelocity();
    inputs.leaderOutput = leaderStatus.getOutput();
    inputs.leaderAppliedVoltage = leaderStatus.getAppliedVoltage();
    inputs.leaderSupplyCurrent = leaderStatus.getSupplyCurrent();
    inputs.leaderStatorCurrent = leaderStatus.getStatorCurrent();
    inputs.leaderTemperature = leaderStatus.getTemperature();

    inputs.followerConnected = followerStatus.getConnected();
    inputs.followerVelocity = followerStatus.getVelocity();
    inputs.leaderOutput = followerStatus.getOutput();
    inputs.followerAppliedVoltage = followerStatus.getAppliedVoltage();
    inputs.followerSupplyCurrent = followerStatus.getSupplyCurrent();
    inputs.followerStatorCurrent = followerStatus.getStatorCurrent();
    inputs.followerTemperature = followerStatus.getTemperature();
  }
}
