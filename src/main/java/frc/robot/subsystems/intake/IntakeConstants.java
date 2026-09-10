package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Voltage;

public final class IntakeConstants {
  /*
   * TODO: Tune constants
   */

  public static final int kPivotMotorId = 1;
  public static final int kRollerMotorId = 2;

  public static final Frequency kStatusFrequency = Hertz.of(50);

  public static final class PivotConfigs {
    public static final double kA = 0.01;
    public static final double kD = 0.15;
    public static final double kI = 0;
    public static final double kP = 1.5;
    public static final double kS = 0.025;
    public static final double kV = 0.25;

    public static final double kMotionMagicCruiseVelocity = 55;
    public static final double kMotionMagicAcceleration = 135;
    public static final double kMotionMagicJerk = 1600;

    public static final MotorOutputConfigs MotorOutputConfigs = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake);

    public static final Slot0Configs Slot0Configs = new Slot0Configs()
        .withKA(kA).withKD(kD).withKI(kI).withKP(kP)
        .withKS(kS).withKV(kV);

    public static final MotionMagicConfigs MotionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(kMotionMagicCruiseVelocity)
        .withMotionMagicAcceleration(kMotionMagicAcceleration)
        .withMotionMagicJerk(kMotionMagicJerk);

    public static final TalonFXConfiguration TalonFXConfiguration = new TalonFXConfiguration()
        .withSlot0(PivotConfigs.Slot0Configs)
        .withMotionMagic(PivotConfigs.MotionMagicConfigs)
        .withMotorOutput(PivotConfigs.MotorOutputConfigs);

    public static final DCMotor kSimMotor = DCMotor.getKrakenX60(1);
    public static final double kSimMotorGearRatio = 1.0;
    public static final double kSimMotorMOIJ = 1.0;
  }

  public static final class RollerConfigs {
    public static final TalonFXConfiguration TalonFXConfiguration = new TalonFXConfiguration();

    public static final DCMotor kSimMotor = DCMotor.getKrakenX60(1);
    public static final double kSimMotorGearRatio = 1.0;
    public static final double kSimMotorMOIJ = 1.0;
  }

  public static final Voltage kIdleVoltage = Volts.zero();
  public static final Voltage kIntakeVoltage = Volts.of(12);
  public static final Voltage kEjectVoltage = Volts.of(-12);

  public static final Angle kStowAngle = Degrees.zero();
  public static final Angle kDeployAngle = Degrees.of(360);

  public enum IntakeRollerState {
    kIdle,
    kIntake,
    kEject;
  }

  public enum IntakePivotState {
    kStow,
    kDeploy;
  }
}
