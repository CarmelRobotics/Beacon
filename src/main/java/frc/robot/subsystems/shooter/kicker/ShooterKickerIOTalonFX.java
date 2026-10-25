package frc.robot.subsystems.shooter.kicker;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.motorstatus.TalonFXStatus;

public class ShooterKickerIOTalonFX implements ShooterKickerIO {
  private final TalonFX motor;
  private final TalonFXStatus motorStatus;
  private final VelocityVoltage velocityRequest;
  private final DutyCycleOut dutyCycleRequest;

  public ShooterKickerIOTalonFX() {
    this.motor = new TalonFX(ShooterConstants.kKickerMotorId);
    this.motorStatus = new TalonFXStatus(motor);
    this.velocityRequest = new VelocityVoltage(0);
    this.dutyCycleRequest = new DutyCycleOut(0);

    motor.getConfigurator().apply(ShooterConstants.KickerConfigs.TalonFXConfiguration);

    motorStatus.setUpdateFrequency(ShooterConstants.kStatusFrequency);
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void updatePID(double kA, double kD, double kI, double kP, double kS, double kV) {
    motor.getConfigurator().apply(new Slot0Configs()
        .withKA(kA).withKD(kD).withKI(kI).withKP(kP).withKS(kS).withKV(kV));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    motor.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void setOutput(double output) {
    motor.setControl(dutyCycleRequest.withOutput(output));
  }

  @Override
  public void updateInputs(ShooterKickerIOInputsAutoLogged inputs) {
    inputs.connected = motorStatus.getConnected();
    inputs.velocity = motorStatus.getVelocity();
    inputs.output = motorStatus.getOutput();
    inputs.appliedVoltage = motorStatus.getAppliedVoltage();
    inputs.supplyCurrent = motorStatus.getSupplyCurrent();
    inputs.statorCurrent = motorStatus.getStatorCurrent();
    inputs.temperature = motorStatus.getTemperature();
  }
}
