package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public final class IntakeConstants {
  public static final int kLeftLeaderPivotMotorId = 13;
  public static final int kRightFollowerPivotMotorId = 21;
  public static final int kLeftLeaderRollerMotorId = 23;
  public static final int kRightFollowerRollerMotorId = 22;

  public static final Frequency kStatusFrequency = Hertz.of(50);

  public static final class PivotConfigs {
    public static final double kS = 0.25;
    public static final double kV = 0.12;
    public static final double kA = 0.01;
    public static final double kP = 4.8;
    public static final double kI = 0.0;
    public static final double kD = 0.1;

    public static final double kMotionMagicCruiseVelocity = 40;
    public static final double kMotionMagicAcceleration = 160;
    public static final double kMotionMagicJerk = 1600;

    public static final MotorOutputConfigs MotorOutputConfigs = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive);

    public static final Slot0Configs Slot0Configs = new Slot0Configs()
        .withKS(kS).withKV(kV).withKA(kA).withKP(kP)
        .withKI(kI).withKD(kD);

    public static final MotionMagicConfigs MotionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(kMotionMagicCruiseVelocity)
        .withMotionMagicAcceleration(kMotionMagicAcceleration)
        .withMotionMagicJerk(kMotionMagicJerk);

    public static final SoftwareLimitSwitchConfigs SoftWareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(Degrees.of(6100));

    public static final TalonFXConfiguration TalonFXConfiguration = new TalonFXConfiguration()
        .withSlot0(PivotConfigs.Slot0Configs)
        .withMotionMagic(PivotConfigs.MotionMagicConfigs)
        .withMotorOutput(PivotConfigs.MotorOutputConfigs)
        .withSoftwareLimitSwitch(SoftWareLimitSwitchConfigs);

    public static final DCMotor kSimMotor = DCMotor.getKrakenX60(1);
    public static final double kSimMotorGearRatio = 11.7675;
    public static final double kSimMotorMOIJ = 0.1;
  }

  public static final class RollerConfigs {
    public static final TalonFXConfiguration TalonFXConfiguration = new TalonFXConfiguration();

    public static final DCMotor kSimMotor = DCMotor.getKrakenX60(1);
    public static final double kSimMotorGearRatio = 2.0;
    public static final double kSimMotorMOIJ = 0.1;
  }

  public static final Voltage kIdleVoltage = Volts.zero();
  public static final Voltage kIntakeVoltage = Volts.of(6.5);
  public static final Voltage kEjectVoltage = Volts.of(-7);

  public static final Angle kStowAngle = Degrees.of(0);
  public static final Angle kAgitatePushAngle = Degrees.of(4200);
  public static final Angle kDeployAngle = Degrees.of(5900);

  public static final Angle kToleranceAngle = Degrees.of(100);
  public static final Time kToleranceTimeout = Seconds.of(0.75);

  public enum IntakeRollerState {
    kIdle,
    kIntake,
    kEject;
  }

  public enum IntakePivotState {
    kStow,
    kDeploy,
    kAgitate;
  }
}
