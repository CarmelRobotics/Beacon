package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface IntakePivotIO {
    @AutoLog
    public static class IntakePivotIOInputs {
        public boolean connected;
        public Angle position;
        public AngularVelocity velocity;
        public Voltage appliedVoltage;
        public Current supplyCurrent;
        public Current statorCurrent;
        public Temperature temperature;
    }

    public void zeroHere();
    public void updatePID(double kA, double kD, double kI, double kP, double kS, double kV);
    public void setAngle(Angle angle);
    public void updateInputs(IntakePivotIOInputsAutoLogged inputs);
}
