package frc.robot.subsystems.shooter.pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface ShooterPivotIO {
  @AutoLog
  public static class ShooterPivotIOInputs {
    public boolean connected;
    public Angle position;
    public AngularVelocity velocity;
    public Voltage appliedVoltage;
    public Current supplyCurrent;
    public Current statorCurrent;
    public Temperature temperature;
  }

  public void zeroHere();

  public void updatePID(double kA, double kD, double kI, double kP, double kS, double kV);

  public default void updateMotionMagic(double kCruiseVelocity, double kAcceleration, double kJerk) {
  }

  public void setPosition(Angle position);

  public void updateInputs(ShooterPivotIOInputsAutoLogged inputs);
}
