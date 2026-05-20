package frc.robot.util;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface MotorStatus {
    public Angle getPosition();
    public AngularVelocity getVelocity();
    public Voltage getAppliedVoltage();
    public Current getSupplyCurrent();
    public Current getStatorCurrent();
    public Temperature getTemperature();
}
