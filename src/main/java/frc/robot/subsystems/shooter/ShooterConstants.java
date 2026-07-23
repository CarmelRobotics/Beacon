package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;

public final class ShooterConstants {
    /*
     * TODO: Tune constants
     */

    public static final int kPivotMotorId = 8;
    public static final int kKickerMotorId = 9;
    public static final int kFrontLeftLeaderRollerMotorId = 4;
    public static final int kBackLeftFollowerRollerMotorId = 5;
    public static final int kFrontRightFollowerRollerMotorId = 6;
    public static final int kBackRightFollowerRollerMotorId = 7;

    public static final Frequency kStatusUpdateFrequency = Hertz.of(60);

    public static final InterpolatingDoubleTreeMap PivotMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap RollerMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap KickerMap = new InterpolatingDoubleTreeMap();
    static {
        // put data here
    }

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

        public static final Slot0Configs Slot0Configs = new Slot0Configs()
            .withKA(kA).withKD(kD).withKI(kI).withKP(kP)
            .withKS(kS).withKV(kV);

        public static final MotionMagicConfigs MotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(kMotionMagicCruiseVelocity)
            .withMotionMagicAcceleration(kMotionMagicAcceleration)
            .withMotionMagicJerk(kMotionMagicJerk);

        public static final TalonFXConfiguration TalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(PivotConfigs.Slot0Configs)
            .withMotionMagic(PivotConfigs.MotionMagicConfigs);

        public static final DCMotor kSimMotor = DCMotor.getKrakenX60(1);
        public static final double kSimMotorGearRatio = 1.0;
        public static final double kSimMotorMOIJ = 1.0;
    }

    public static final class RollerConfigs {
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
    public static final AngularVelocity kKickerStaticScoreVelocity = RotationsPerSecond.of(4.5);

    public static final Angle kPivotStowAngle = Degrees.zero();
    public static final Angle kPivotStaticScoreAngle = Degrees.of(5);

    public static final AngularVelocity kRollerIdleVelocity = RotationsPerSecond.of(0.5);
    public static final AngularVelocity kRollerStaticScoreVelocity = RotationsPerSecond.of(40);
    public static final double kRollerSendOutput = 1.0;

    public enum KickerState {
        IDLE,
        EJECT,
        STATIC_SCORE,
        PASS,
        SCORE;
    }

    public enum PivotState {
        STOW,
        STATIC_SCORE,
        PASS,
        SCORE;
    }

    public enum RollerState {
        IDLE,
        STATIC_SCORE,
        STATIC_SEND,
        PASS,
        SCORE;
    }
}
