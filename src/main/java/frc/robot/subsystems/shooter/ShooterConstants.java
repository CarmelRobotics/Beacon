package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;

public final class ShooterConstants {
  public static final int kPivotMotorId = 14;
  public static final int kKickerMotorId = 20;
  public static final int kFrontLeftLeaderRollerMotorId = 16;
  public static final int kBackLeftFollowerRollerMotorId = 17;
  public static final int kFrontRightFollowerRollerMotorId = 19;
  public static final int kBackRightFollowerRollerMotorId = 18;

  public static final Frequency kStatusFrequency = Hertz.of(50);

  /* Distance in Meters -> Degrees */
  public static final InterpolatingDoubleTreeMap PivotScoringMap = new InterpolatingDoubleTreeMap();
  /* Distance in Meters -> RotationsPerSecond */
  public static final InterpolatingDoubleTreeMap RollerPassingMap = new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap RollerScoringMap = new InterpolatingDoubleTreeMap();

  static {
    PivotScoringMap.put(1.3, 0.0);
    PivotScoringMap.put(2.0, 8.0);
    PivotScoringMap.put(3.0, 17.0);
    PivotScoringMap.put(4.0, 23.0);
    PivotScoringMap.put(5.0, 28.0);

    RollerPassingMap.put(2.0, 33.33);
    RollerPassingMap.put(6.0, 58.33);
    RollerPassingMap.put(10.0, 100.0);

    RollerScoringMap.put(1.3, 35.0);
    RollerScoringMap.put(2.0, 37.5);
    RollerScoringMap.put(3.0, 40.0);
    RollerScoringMap.put(4.0, 43.33);
    RollerScoringMap.put(5.0, 48.33);
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

    public static final SoftwareLimitSwitchConfigs SoftwareLimitSwitchConfigs = 
      new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
      .withForwardSoftLimitThreshold(Degrees.of(900));

    public static final TalonFXConfiguration TalonFXConfiguration = new TalonFXConfiguration()
        .withSlot0(PivotConfigs.Slot0Configs)
        .withMotionMagic(PivotConfigs.MotionMagicConfigs)
        .withSoftwareLimitSwitch(SoftwareLimitSwitchConfigs);

    public static final DCMotor kSimMotor = DCMotor.getKrakenX60(1);
    public static final double kSimMotorGearRatio = 1.0;
    public static final double kSimMotorMOIJ = 1.0;
  }

  public static final class RollerConfigs {
    public static final double kA = 0.0;
    public static final double kD = 0.0;
    public static final double kI = 0;
    public static final double kP = 0.5;
    public static final double kS = 0.02;
    public static final double kV = 0.486;

    public static final Slot0Configs Slot0Configs = new Slot0Configs()
        .withKA(kA).withKD(kD).withKI(kI).withKP(kP)
        .withKS(kS).withKV(kV);

    public static final TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration()
        .withSlot0(RollerConfigs.Slot0Configs);

    public static final DCMotor kSimMotor = DCMotor.getKrakenX60(1);
    public static final double kSimMotorGearRatio = 2.0;
    public static final double kSimMotorMOIJ = 1.0;
  }

  public static final class KickerConfigs {   
    public static final double kA = 0.01;
    public static final double kD = 0.15;
    public static final double kI = 0;
    public static final double kP = 1.5;
    public static final double kS = 0.025;
    public static final double kV = 0.25;

    public static final Slot0Configs Slot0Configs = new Slot0Configs()
        .withKA(kA).withKD(kD).withKI(kI).withKP(kP)
        .withKS(kS).withKV(kV);

    public static final TalonFXConfiguration TalonFXConfiguration = new TalonFXConfiguration()
        .withSlot0(RollerConfigs.Slot0Configs);

    public static final DCMotor kSimMotor = DCMotor.getKrakenX60(1);
    public static final double kSimMotorGearRatio = 1.0;
    public static final double kSimMotorMOIJ = 1.0;
  }

  public static final double kKickerIdleOutput = 0.0;
  public static final double kKickerEjectOutput = -1.0;
  public static final AngularVelocity kKickerFeedVelocity = RotationsPerSecond.of(4.5);

  public static final Angle kPivotStowAngle = Degrees.zero();
  public static final Angle kPivotStaticScoreAngle = Degrees.of(100);
  public static final Angle kPivotPassAngle = Degrees.of(600);

  public static final AngularVelocity kRollerIdleVelocity = RotationsPerSecond.of(0.5);
  public static final AngularVelocity kRollerStaticScoreVelocity = RotationsPerSecond.of(40);

  public static final AngularVelocity kRollerToleranceVelocity = RotationsPerSecond.of(1);

  public enum ShooterKickerState {
    kIdle,
    kEject,
    kFeed;
  }

  public enum ShooterPivotState {
    kStow,
    kStaticScore,
    kPass,
    kScore;
  }

  public enum ShooterRollerState {
    kIdle,
    kStaticScore,
    kPass,
    kScore;
  }
}
