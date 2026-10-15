package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static final Mode kSimulationMode = Mode.kSim;
  public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.kReal : kSimulationMode;
  public static final boolean kDisableHAL = kCurrentMode == Mode.kReplay;
  public static final boolean kUseStaticSetpoints = false;

  public enum Mode {
    kReal,
    kSim,
    kReplay;
  }

  public static final double kTimeoutSeconds = 0.25;
  public static final int kMaxAttempts = 5;
}
