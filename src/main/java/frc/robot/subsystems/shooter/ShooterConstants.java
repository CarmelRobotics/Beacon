// Copyright (c) 2025-2026 Rockin' Bots
// https://github.com/CarmelRobotics

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Voltage;

public final class ShooterConstants {
  public static final int kPivotMotorId = 14;
  public static final int kKickerMotorId = 20;
  public static final int kFrontLeftLeaderRollerMotorId = 16;
  public static final int kBackLeftFollowerRollerMotorId = 17;
  public static final int kFrontRightFollowerRollerMotorId = 19;
  public static final int kBackRightFollowerRollerMotorId = 18;

  public static final Frequency kStatusFrequency = Hertz.of(50);

  /* Distance in Meters -> Rotations Per Second */
  public static final InterpolatingDoubleTreeMap RollerScoringVelocityMap = new InterpolatingDoubleTreeMap();

  static {
    RollerScoringVelocityMap.put(2.0, 25.0);
    RollerScoringVelocityMap.put(2.770, 27.0);
    RollerScoringVelocityMap.put(3.140, 30.0);
    RollerScoringVelocityMap.put(3.650, 34.0);
  }

  public static final class PivotConfigs {
    public static final double kA = 0.01;
    public static final double kD = 0.15;
    public static final double kI = 0;
    public static final double kP = 1.5;
    public static final double kS = 0.025;
    public static final double kV = 0.25;

    public static final double kMotionMagicCruiseVelocity = 20;
    public static final double kMotionMagicAcceleration = 135;
    public static final double kMotionMagicJerk = 1600;

    public static final Slot0Configs Slot0Configs = new Slot0Configs()
        .withKA(kA).withKD(kD).withKI(kI).withKP(kP)
        .withKS(kS).withKV(kV);

    public static final MotionMagicConfigs MotionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(kMotionMagicCruiseVelocity)
        .withMotionMagicAcceleration(kMotionMagicAcceleration)
        .withMotionMagicJerk(kMotionMagicJerk);

    public static final SoftwareLimitSwitchConfigs SoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(Degrees.of(1200));

    public static final TalonFXConfiguration TalonFXConfiguration = new TalonFXConfiguration()
        .withSlot0(PivotConfigs.Slot0Configs)
        .withMotionMagic(PivotConfigs.MotionMagicConfigs)
        .withSoftwareLimitSwitch(PivotConfigs.SoftwareLimitSwitchConfigs);

    public static final DCMotor kSimMotor = DCMotor.getKrakenX60(1);
    public static final double kSimMotorGearRatio = 1.0;
    public static final double kSimMotorMOIJ = 1.0;
  }

  public static final class RollerConfigs {
    public static final double kA = 0.0;
    public static final double kD = 0.0;
    public static final double kI = 0.0;
    public static final double kP = 0.0;
    public static final double kS = 0.38;
    public static final double kV = 0.24;

    public static final Slot0Configs Slot0Configs = new Slot0Configs()
      .withKA(kA).withKD(kD).withKI(kI).withKP(kP)
      .withKS(kS).withKV(kV);

    public static final FeedbackConfigs FeedbackConfigs = new FeedbackConfigs()
      .withSensorToMechanismRatio(2.1333);

    public static final CurrentLimitsConfigs CurrentLimitConfigs = new CurrentLimitsConfigs()
      .withSupplyCurrentLimit(Amps.of(60))
      .withSupplyCurrentLimitEnable(true);

    public static final TalonFXConfiguration TalonFXConfiguration = new TalonFXConfiguration()
        .withSlot0(RollerConfigs.Slot0Configs)
        .withFeedback(RollerConfigs.FeedbackConfigs)
        .withCurrentLimits(RollerConfigs.CurrentLimitConfigs);

    public static final DCMotor kSimMotor = DCMotor.getKrakenX60(1);
    public static final double kSimMotorGearRatio = 2.0;
    public static final double kSimMotorMOIJ = 1.0;
  }

  public static final class KickerConfigs {
    public static final TalonFXConfiguration TalonFXConfiguration = new TalonFXConfiguration();

    public static final DCMotor kSimMotor = DCMotor.getKrakenX60(1);
    public static final double kSimMotorGearRatio = 1.0;
    public static final double kSimMotorMOIJ = 1.0;
  }

  public static final Voltage kKickerIdleVoltage = Volts.of(0);
  public static final Voltage kKickerEjectVoltage = Volts.of(12.0);
  public static final Voltage kKickerFeedVoltage = Volts.of(-12.0);

  public static final Angle kPivotStowAngle = Degrees.zero();
  public static final Angle kPivotScoreAngle = Degrees.of(0);
  public static final Angle kPivotPassAngle = Degrees.of(550);

  public static final Voltage kRollerIdleVoltage = Volts.zero();
  public static final AngularVelocity kRollerPassVelocity = RotationsPerSecond.of(34);
  public static final AngularVelocity kRollerScoreVelocity = RotationsPerSecond.of(27);

  public static final AngularVelocity kRollerToleranceVelocity = RotationsPerSecond.of(3);

  public enum ShooterKickerState {
    kIdle,
    kEject,
    kFeed;
  }

  public enum ShooterPivotState {
    kStow,
    kPass,
    kScore;
  }

  public enum ShooterRollerState {
    kIdle,
    kPass,
    kStaticScore,
    kScore;
  }
}
