package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Frequency;

public final class ShooterConstants {
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
        public static final double kSimMotorGearRatio = 1.0; // Please Tune
        public static final double kSimMotorMOIJ = 1.0; // Please Tune
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
        public static final double kSimMotorGearRatio = 1.0; // Please Tune
        public static final double kSimMotorMOIJ = 1.0; // Please Tune
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
        public static final double kSimMotorGearRatio = 1.0; // Please Tune
        public static final double kSimMotorMOIJ = 1.0; // Please Tune
    }

    public enum ShooterState {
        STOW,
        OFF,
        PASS_PRESET,
        PASS,
        SCORE_PRESET,
        SCORE;
    }
}
