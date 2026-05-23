package frc.robot.subsystems.shooter.kicker;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface ShooterKickerIO {
    @AutoLog
    public static class ShooterKickerIOInputs {
        public boolean connected;
        public AngularVelocity velocity;
        public Voltage appliedVoltage;
        public Current supplyCurrent;
        public Current statorCurrent;
        public Temperature temperature;
    }

    public void updatePID(double kA, double kD, double kI, double kP, double kS, double kV);
    public void setVelocity(AngularVelocity velocity);
    public void setVoltage(Voltage voltage);
    public void updateInputs(ShooterKickerIOInputsAutoLogged inputs);
}
