package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Hertz;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Frequency;

public final class ShooterConstants {
    public static final Frequency kStatusUpdateFrequency = Hertz.of(60);

    public static final InterpolatingDoubleTreeMap PivotMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap RollerMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap KickerMap = new InterpolatingDoubleTreeMap();
    static {
        // put data here
    }

    public static final class PivotConfigs {}

    public static final class RollerConfigs {}

    public static final class KickerConfigs {}
}
