package frc.robot.util.motorstatus;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface MotorStatus {
  public boolean getConnected();

  public Angle getPosition();

  public Double getOutput();

  public AngularVelocity getVelocity();

  public Voltage getAppliedVoltage();

  public Current getSupplyCurrent();

  public Current getStatorCurrent();

  public Temperature getTemperature();
}
