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
import frc.robot.util.motorstatus.TalonFXStatus;

public class IntakePivotIOTalonFX implements IntakePivotIO {
  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;

  private final TalonFXStatus leaderStatus;
  private final TalonFXStatus followerStatus;

  private final MotionMagicVoltage positionRequest;

  public IntakePivotIOTalonFX() {
    this.leaderMotor = new TalonFX(IntakeConstants.kLeftLeaderPivotMotorId, Constants.kRIOCANBus);
    this.followerMotor = new TalonFX(IntakeConstants.kRightFollowerPivotMotorId, Constants.kRIOCANBus);

    this.leaderStatus = new TalonFXStatus(leaderMotor);
    this.followerStatus = new TalonFXStatus(followerMotor);

    this.positionRequest = new MotionMagicVoltage(0);

    leaderMotor.getConfigurator().apply(IntakeConstants.PivotConfigs.TalonFXConfiguration);
    followerMotor.getConfigurator().apply(IntakeConstants.PivotConfigs.TalonFXConfiguration);
    followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    leaderStatus.setUpdateFrequency(IntakeConstants.kStatusFrequency);
    followerStatus.setUpdateFrequency(IntakeConstants.kStatusFrequency);
    ParentDevice.optimizeBusUtilizationForAll(leaderMotor, followerMotor);
    leaderMotor.setControl(positionRequest);
  }

  @Override
  public void zeroHere() {
    leaderMotor.setPosition(0);
    followerMotor.setPosition(0);
  }

  @Override
  public void updatePID(double kA, double kD, double kI, double kP, double kS, double kV) {
    final Slot0Configs config = new Slot0Configs()
      .withKA(kA).withKD(kD).withKI(kI).withKP(kP).withKS(kS).withKV(kV);

    leaderMotor.getConfigurator().apply(config);
    followerMotor.getConfigurator().apply(config);
  }

  public void updateMotionMagic(double kCruiseVelocity, double kAcceleration, double kJerk) {
    final MotionMagicConfigs config = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(kCruiseVelocity)
      .withMotionMagicAcceleration(kAcceleration)
      .withMotionMagicJerk(kJerk);

    leaderMotor.getConfigurator().apply(config);
    followerMotor.getConfigurator().apply(config);
  }

  @Override
  public void setPosition(Angle position) {
    leaderMotor.setControl(positionRequest.withPosition(position));
  }

  @Override
  public void updateInputs(IntakePivotIOInputsAutoLogged inputs) {
    leaderStatus.refresh();
    followerStatus.refresh();

    inputs.leaderConnected = leaderStatus.getConnected();
    inputs.leaderPosition = leaderStatus.getPosition();
    inputs.leaderVelocity = leaderStatus.getVelocity();
    inputs.leaderAppliedVoltage = leaderStatus.getAppliedVoltage();
    inputs.leaderStatorCurrent = leaderStatus.getSupplyCurrent();
    inputs.leaderTemperature = leaderStatus.getTemperature();

    inputs.followerConnected = followerStatus.getConnected();
    inputs.followerPosition = followerStatus.getPosition();
    inputs.followerVelocity = followerStatus.getVelocity();
    inputs.followerAppliedVoltage = followerStatus.getAppliedVoltage();
    inputs.followerStatorCurrent = followerStatus.getSupplyCurrent();
    inputs.followerTemperature = followerStatus.getTemperature();
  }
}
