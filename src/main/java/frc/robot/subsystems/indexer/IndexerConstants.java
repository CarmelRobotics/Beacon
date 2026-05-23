package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Frequency;

public final class IndexerConstants {
    public static final int kMotorId = 3;

    public static final Frequency kStatusUpdateFrequency = Hertz.of(60);

    public static final class Configs {
        public static final TalonFXConfiguration TalonFXConfiguration = new TalonFXConfiguration();

        public static final DCMotor kSimMotor = DCMotor.getKrakenX60(1);
        public static final double kSimMotorGearRatio = 1.0; // Please Tune
        public static final double kSimMotorMOIJ = 1.0; // Please Tune
    }
}
