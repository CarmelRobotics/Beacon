package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.motorstatus.TalonFXStatus;

public class IntakePivotIOTalonFX implements IntakePivotIO {
  private final TalonFX motor;
  private final TalonFXStatus motorStatus;

  private final MotionMagicVoltage positionRequest;

  public IntakePivotIOTalonFX() {
    this.motor = new TalonFX(IntakeConstants.kPivotMotorId);
    this.motorStatus = new TalonFXStatus(motor);
    this.positionRequest = new MotionMagicVoltage(0);

    motor.getConfigurator().apply(IntakeConstants.PivotConfigs.TalonFXConfiguration);
    motorStatus.setUpdateFrequency(IntakeConstants.kStatusFrequency);
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void zeroHere() {
    motor.setPosition(0);
  }

  @Override
  public void updatePID(double kA, double kD, double kI, double kP, double kS, double kV) {
    motor.getConfigurator().apply(new Slot0Configs()
        .withKA(kA).withKD(kD).withKI(kI).withKP(kP).withKS(kS).withKV(kV));
  }

  public void updateMotionMagic(double kCruiseVelocity, double kAcceleration, double kJerk) {
    motor.getConfigurator().apply(new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(kCruiseVelocity)
        .withMotionMagicAcceleration(kAcceleration)
        .withMotionMagicJerk(kJerk));
  }

  @Override
  public void setPosition(Angle position) {
    motor.setControl(positionRequest.withPosition(position));
  }

  @Override
  public void updateInputs(IntakePivotIOInputsAutoLogged inputs) {
    inputs.connected = motorStatus.getConnected();
    inputs.position = motorStatus.getPosition();
    inputs.velocity = motorStatus.getVelocity();
    inputs.appliedVoltage = motorStatus.getAppliedVoltage();
    inputs.statorCurrent = motorStatus.getSupplyCurrent();
    inputs.temperature = motorStatus.getTemperature();
  }
}
