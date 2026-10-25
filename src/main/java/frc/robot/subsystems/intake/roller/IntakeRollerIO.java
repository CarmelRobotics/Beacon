package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface IntakeRollerIO {
  @AutoLog
  public static class IntakeRollerIOInputs {
    public boolean leaderConnected;
    public AngularVelocity leaderVelocity;
    public double leaderOutput;
    public Voltage leaderAppliedVoltage;
    public Current leaderSupplyCurrent;
    public Current leaderStatorCurrent;
    public Temperature leaderTemperature;

    public boolean followerConnected;
    public AngularVelocity followerVelocity;
    public double followerOutput;
    public Voltage followerAppliedVoltage;
    public Current followerSupplyCurrent;
    public Current followerStatorCurrent;
    public Temperature followerTemperature;
  }

  public default void setVoltage(Voltage voltage) {}

  public default void updateInputs(IntakeRollerIOInputs inputs) {}
}
