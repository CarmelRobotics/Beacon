package frc.robot.util.motorstatus;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXStatus implements MotorStatus {
  private final TalonFX motor;
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Double> output;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temperature;

  public TalonFXStatus(final TalonFX motor) {
    this.motor = motor;
    this.position = motor.getPosition();
    this.velocity = motor.getVelocity();
    this.output = motor.getDutyCycle();
    this.appliedVoltage = motor.getMotorVoltage();
    this.supplyCurrent = motor.getSupplyCurrent();
    this.statorCurrent = motor.getStatorCurrent();
    this.temperature = motor.getDeviceTemp();
  }

  public void setUpdateFrequency(Frequency frequency) {
    BaseStatusSignal.setUpdateFrequencyForAll(
        frequency, position, velocity, output,
        appliedVoltage, supplyCurrent, statorCurrent, temperature);
  }

  public void refresh() {
    BaseStatusSignal.refreshAll(
        position, velocity, output, appliedVoltage,
        supplyCurrent, statorCurrent, temperature);
  }

  @Override
  public boolean getConnected() {
    return motor.isConnected();
  }

  @Override
  public Angle getPosition() {
    return position.getValue();
  }

  @Override
  public AngularVelocity getVelocity() {
    return velocity.getValue();
  }

  @Override
  public Double getOutput() {
    return output.getValue();
  }

  @Override
  public Voltage getAppliedVoltage() {
    return appliedVoltage.getValue();
  }

  @Override
  public Current getSupplyCurrent() {
    return supplyCurrent.getValue();
  }

  @Override
  public Current getStatorCurrent() {
    return statorCurrent.getValue();
  }

  @Override
  public Temperature getTemperature() {
    return temperature.getValue();
  }
}
