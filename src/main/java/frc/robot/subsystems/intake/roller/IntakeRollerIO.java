package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface IntakeRollerIO {
  @AutoLog
  public static class IntakeRollerIOInputs {
    public boolean connected;
    public AngularVelocity velocity;
    public Voltage appliedVoltage;
    public Current supplyCurrent;
    public Current statorCurrent;
    public Temperature temperature;
  }

  public void setVoltage(Voltage voltage);

  public void updateInputs(IntakeRollerIOInputs inputs);
}
