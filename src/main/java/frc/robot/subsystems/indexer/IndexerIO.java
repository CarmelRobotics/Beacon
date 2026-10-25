package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean connected;
    public AngularVelocity velocity;
    public double output;
    public Voltage appliedVoltage;
    public Current supplyCurrent;
    public Current statorCurrent;
    public Temperature temperature;
  }

  public default void setVoltage(Voltage voltage) {}

  public default void updateInputs(IndexerIOInputsAutoLogged inputs) {}
}
