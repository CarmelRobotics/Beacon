package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.motorstatus.TalonFXStatus;

public class IntakePivotIOTalonFX implements IntakePivotIO {
  private final TalonFX leftLeaderMotor;
  private final TalonFX rightFollowerMotor;

  private final TalonFXStatus leaderStatus;
  private final TalonFXStatus followerStatus;

  private final MotionMagicVoltage positionRequest;

  public IntakePivotIOTalonFX() {
    this.leftLeaderMotor = new TalonFX(IntakeConstants.kLeftLeaderPivotMotorId);
    this.rightFollowerMotor = new TalonFX(IntakeConstants.kRightFollowerPivotMotorId);

    this.leaderStatus = new TalonFXStatus(leftLeaderMotor);
    this.followerStatus = new TalonFXStatus(rightFollowerMotor);

    this.positionRequest = new MotionMagicVoltage(0);

    PhoenixUtil.tryUntilOk(Constants.kMaxAttempts,
      () -> leftLeaderMotor.getConfigurator().apply(IntakeConstants.PivotConfigs.TalonFXConfiguration, Constants.kTimeoutSeconds));
    PhoenixUtil.tryUntilOk(Constants.kMaxAttempts,
      () -> rightFollowerMotor.getConfigurator().apply(IntakeConstants.PivotConfigs.TalonFXConfiguration, Constants.kTimeoutSeconds));
    
    rightFollowerMotor.setControl(new Follower(leftLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    leaderStatus.setUpdateFrequency(IntakeConstants.kStatusFrequency);
    followerStatus.setUpdateFrequency(IntakeConstants.kStatusFrequency);
    ParentDevice.optimizeBusUtilizationForAll(leftLeaderMotor, rightFollowerMotor);
    leftLeaderMotor.setControl(positionRequest);
  }

  @Override
  public void zeroHere() {
    leftLeaderMotor.setPosition(0);
    rightFollowerMotor.setPosition(0);
  }

  @Override
  public void updatePID(double kA, double kD, double kI, double kP, double kS, double kV) {
    final Slot0Configs config = new Slot0Configs()
      .withKA(kA).withKD(kD).withKI(kI).withKP(kP).withKS(kS).withKV(kV);

    leftLeaderMotor.getConfigurator().apply(config);
    rightFollowerMotor.getConfigurator().apply(config);
  }

  public void updateMotionMagic(double kCruiseVelocity, double kAcceleration, double kJerk) {
    final MotionMagicConfigs config = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(kCruiseVelocity)
      .withMotionMagicAcceleration(kAcceleration)
      .withMotionMagicJerk(kJerk);

    leftLeaderMotor.getConfigurator().apply(config);
    rightFollowerMotor.getConfigurator().apply(config);
  }

  @Override
  public void setPosition(Angle position) {
    leftLeaderMotor.setControl(positionRequest.withPosition(position));
  }

  @Override
  public void updateInputs(IntakePivotIOInputsAutoLogged inputs) {
    leaderStatus.refresh();
    followerStatus.refresh();

    inputs.leaderConnected = leaderStatus.getConnected();
    inputs.leaderPosition = leaderStatus.getPosition();
    inputs.leaderVelocity = leaderStatus.getVelocity();
    inputs.leaderOutput = leaderStatus.getOutput();
    inputs.leaderAppliedVoltage = leaderStatus.getAppliedVoltage();
    inputs.leaderStatorCurrent = leaderStatus.getSupplyCurrent();
    inputs.leaderTemperature = leaderStatus.getTemperature();

    inputs.followerConnected = followerStatus.getConnected();
    inputs.followerPosition = followerStatus.getPosition();
    inputs.followerVelocity = followerStatus.getVelocity();
    inputs.followerOutput = followerStatus.getOutput();
    inputs.followerAppliedVoltage = followerStatus.getAppliedVoltage();
    inputs.followerStatorCurrent = followerStatus.getSupplyCurrent();
    inputs.followerTemperature = followerStatus.getTemperature();
  }
}
