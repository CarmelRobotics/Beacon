package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static final Mode kSimulationMode = Mode.SIM;
  public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : kSimulationMode;
  public static final boolean kDisableHAL = kCurrentMode == Mode.REPLAY;

  public enum Mode {
    REAL,
    SIM,
    REPLAY;
  }

  public static final double kTimeoutSeconds = 0.25;
  public static final int kMaxAttempts = 5;
}
