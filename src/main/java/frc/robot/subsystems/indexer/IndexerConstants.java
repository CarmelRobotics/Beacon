// Copyright (c) 2025-2026 Rockin' Bots
// https://github.com/CarmelRobotics

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public final class IndexerConstants {
  public static final int kMotorId = 15;

  public static final Frequency kStatusFrequency = Hertz.of(50);

  public static final Voltage kIdleVoltage = Volts.zero();
  public static final Voltage kIntakeVoltage = Volts.of(-4);
  public static final Voltage kFeedVoltage = Volts.of(-12);
  public static final Voltage kEjectVoltage = Volts.of(5);

  public static final Time kAgitateTimeout = Seconds.of(0.5);

  public enum IndexerState {
    kIdle,
    kEject,
    kFeed;
  }

  public static final class Configs {
    public static final CurrentLimitsConfigs CurrentLimitConfigs = new CurrentLimitsConfigs()
      .withSupplyCurrentLimit(Amps.of(30))
      .withSupplyCurrentLimitEnable(true);

    public static final TalonFXConfiguration TalonFXConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(CurrentLimitConfigs);

    public static final DCMotor kSimMotor = DCMotor.getKrakenX60(1);
    public static final double kSimMotorGearRatio = 1.0;
    public static final double kSimMotorMOIJ = 1.0;
  }
}
