package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Voltage;

public final class IndexerConstants {
  /*
   * TODO: Tune constants
   */

  public static final int kMotorId = 3;

  public static final Frequency kStatusFrequency = Hertz.of(50);

  public static final Voltage kIdleVoltage = Volts.zero();
  public static final Voltage kFeedVoltage = Volts.of(12);
  public static final Voltage kEjectVoltage = Volts.of(-12);

  public enum IndexerState {
    kIdle,
    kFeed,
    kEject;
  }

  public static final class Configs {
    public static final TalonFXConfiguration TalonFXConfiguration = new TalonFXConfiguration();

    public static final DCMotor kSimMotor = DCMotor.getKrakenX60(1);
    public static final double kSimMotorGearRatio = 1.0;
    public static final double kSimMotorMOIJ = 1.0;
  }
}
