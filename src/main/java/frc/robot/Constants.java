package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static final Mode kSimulationMode = Mode.kSim;
  public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.kReal : kSimulationMode;
  public static final boolean kDisableHAL = kCurrentMode == Mode.kReplay;
  public static final Time kRollerDelay = Seconds.of(1.0);

  public enum Mode {
    kReal,
    kSim,
    kReplay;
  }

  public static final double kTimeoutSeconds = 0.25;
  public static final int kMaxAttempts = 5;
}
