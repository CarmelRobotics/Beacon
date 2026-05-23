package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean pivotConnected;
        public Angle pivotPosition;
        public AngularVelocity pivotVelocity;
        public Voltage pivotAppliedVoltage;
        public Current pivotSupplyCurrent;
        public Current pivotStatorCurrent;
        public Temperature pivotTemperature;

        public boolean rollerConnected;
        public AngularVelocity rollerVelocity;
        public Voltage rollerAppliedVoltage;
        public Current rollerSupplyCurrent;
        public Current rollerStatorCurrent;
        public Temperature rollerTemperature;
    }

    public enum PivotState {
        STOW(Degrees.zero()),
        GROUND(Degrees.of(360));

        public final Angle position;

        private PivotState(Angle position) {
            this.position = position;
        }
    }

    public enum RollerState {
        OFF(Volts.zero()),
        INTAKE(Volts.of(12)),
        EJECT(Volts.of(-12));

        public final Voltage voltage;

        private RollerState(Voltage voltage) {
            this.voltage = voltage;
        }
    }

    public void zeroHere();
    public void updatePivotPID(double kA, double kD, double kI, double kP, double kS, double kV);
    public default void updateMotionMagic(double kCruiseVelocity, double kAcceleration, double kJerk) {}
    public void setPivotPosition(Angle position);
    public void setRollerVoltage(Voltage voltage);

    public void updateInputs(IntakeIOInputs inputs);
}
