// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive.module;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.PhoenixOdometryThread;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.motorstatus.TalonFXStatus;

public class ModuleIOTalonFX implements ModuleIO {
  private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANcoder encoder;

  private final TalonFXStatus driveMotorStatus;
  private final TalonFXStatus steerMotorStatus;
  private final StatusSignal<MagnetHealthValue> encoderMagnetHealth;
  private final StatusSignal<Angle> encoderPosition;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> steerPositionQueue;

  private final VoltageOut voltageRequest;
  private final VelocityVoltage velocityRequest;
  private final MotionMagicExpoVoltage positionRequest;

  public ModuleIOTalonFX(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
    this.constants = constants;

    this.driveMotor = new TalonFX(constants.DriveMotorId, DriveConstants.kCANBus);
    this.steerMotor = new TalonFX(constants.SteerMotorId, DriveConstants.kCANBus);
    this.encoder = new CANcoder(constants.EncoderId, DriveConstants.kCANBus);

    this.driveMotorStatus = new TalonFXStatus(driveMotor);
    this.steerMotorStatus = new TalonFXStatus(steerMotor);
    this.encoderMagnetHealth = encoder.getMagnetHealth();
    this.encoderPosition = encoder.getPosition();

    this.voltageRequest = new VoltageOut(0);
    this.velocityRequest = new VelocityVoltage(0);
    this.positionRequest = new MotionMagicExpoVoltage(0);

    this.timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    this.drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveMotor.getPosition().clone());
    this.steerPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(steerMotor.getPosition().clone());

    configureModule();

    driveMotorStatus.setUpdateFrequency(DriveConstants.kStatusFrequency);
    steerMotorStatus.setUpdateFrequency(DriveConstants.kStatusFrequency);
    BaseStatusSignal.setUpdateFrequencyForAll(DriveConstants.kStatusFrequency, encoderMagnetHealth, encoderPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(DriveConstants.kOdometryFrequency, driveMotor.getPosition(),
        steerMotor.getPosition());
    ParentDevice.optimizeBusUtilizationForAll(driveMotor, steerMotor, encoder);
  }

  private void configureModule() {
    final TalonFXConfiguration driveConfig = constants.DriveMotorInitialConfigs;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = constants.DriveMotorGains;
    driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted = constants.DriveMotorInverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;

    final TalonFXConfiguration steerConfig = constants.SteerMotorInitialConfigs;
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.Slot0 = constants.SteerMotorGains;
    steerConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    steerConfig.Feedback.FeedbackSensorSource = DriveConstants.kFeedbackSensorSource;
    steerConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    steerConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
    steerConfig.MotionMagic.MotionMagicAcceleration = steerConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    steerConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
    steerConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
    steerConfig.MotorOutput.Inverted = constants.SteerMotorInverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;

    final CANcoderConfiguration encoderConfig = constants.EncoderInitialConfigs;
    encoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
    encoderConfig.MagnetSensor.SensorDirection = constants.EncoderInverted
        ? SensorDirectionValue.Clockwise_Positive
        : SensorDirectionValue.CounterClockwise_Positive;

    PhoenixUtil.tryUntilOk(Constants.kMaxAttempts,
        () -> driveMotor.getConfigurator().apply(driveConfig, Constants.kTimeoutSeconds));
    PhoenixUtil.tryUntilOk(Constants.kMaxAttempts,
        () -> steerMotor.getConfigurator().apply(steerConfig, Constants.kTimeoutSeconds));
    PhoenixUtil.tryUntilOk(Constants.kMaxAttempts, () -> encoder.getConfigurator().apply(encoderConfig));
  }

  @Override
  public void setDriveVoltage(Voltage voltage) {
    driveMotor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setSteerVoltage(Voltage voltage) {
    steerMotor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    driveMotor.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void setSteer(Rotation2d rotation) {
    steerMotor.setControl(positionRequest.withPosition(rotation.getRotations()));
  }

  @Override
  public void updateInputs(ModuleIOInputsAutoLogged inputs) {
    inputs.driveConnected = driveMotorStatus.getConnected();
    inputs.drivePosition = driveMotorStatus.getPosition();
    inputs.driveVelocity = driveMotorStatus.getVelocity();
    inputs.driveAppliedVoltage = driveMotorStatus.getAppliedVoltage();
    inputs.driveSupplyCurrent = driveMotorStatus.getSupplyCurrent();
    inputs.driveStatorCurrent = driveMotorStatus.getStatorCurrent();
    inputs.driveTemperature = driveMotorStatus.getTemperature();

    inputs.steerConnected = steerMotorStatus.getConnected();
    inputs.steerPosition = new Rotation2d(steerMotorStatus.getPosition());
    inputs.steerVelocity = steerMotorStatus.getVelocity();
    inputs.steerAppliedVoltage = steerMotorStatus.getAppliedVoltage();
    inputs.steerSupplyCurrent = steerMotorStatus.getSupplyCurrent();
    inputs.steerStatorCurrent = steerMotorStatus.getStatorCurrent();
    inputs.steerTemperature = steerMotorStatus.getTemperature();

    inputs.encoderConnected = encoder.isConnected();
    inputs.encoderMagnetIsOk = encoderMagnetHealth.getValue() == MagnetHealthValue.Magnet_Green;
    inputs.encoderPosition = new Rotation2d(encoderPosition.getValue());

    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRads = drivePositionQueue.stream()
        .mapToDouble((Double position) -> Units.rotationsToRadians(position))
        .toArray();
    inputs.odometrySteerPositions = steerPositionQueue.stream()
        .map((Double position) -> Rotation2d.fromRotations(position.doubleValue()))
        .toArray(Rotation2d[]::new);

    timestampQueue.clear();
    drivePositionQueue.clear();
    steerPositionQueue.clear();
  }
}
