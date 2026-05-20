package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Frequency;

public final class IntakeConstants {
    public static final int kPivotMotorId = 1;
    public static final int kRollerMotorId = 2;

    public static final Frequency kStatusUpdateFrequency = Hertz.of(60);
    
    public static final DCMotor kSimPivotMotor = DCMotor.getKrakenX60(1);
    public static final double kSimPivotMotorGearRatio = 1.0; // Please Tune
    public static final double kSimPivotMotorGearMOIJ = 1.0; // Please Tune

    public static final DCMotor kSimRollerMotor = DCMotor.getKrakenX60(1);
    public static final double kSimRollerMotorGearRatio = 1.0; // Please Tune
    public static final double kSimRollerMotorGearMOIJ = 1.0; // Please Tune

    private static final class PivotConfigs {
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
    }

    public static final TalonFXConfiguration PivotConfiguration = new TalonFXConfiguration()
        .withSlot0(PivotConfigs.Slot0Configs)
        .withMotionMagic(PivotConfigs.MotionMagicConfigs);
    
    public static final TalonFXConfiguration RollerConfiguration = new TalonFXConfiguration();
}
