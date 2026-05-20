package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputsAutoLogged {
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
        STOW,
        GROUND;
    }

    public enum RollerState {
        OFF,
        INTAKE,
        EJECT;
    }

    public static class IntakeIOOutputs {
        public PivotState pivotState = PivotState.STOW;
        public RollerState rollerState = RollerState.OFF;

        public Angle setPivotPosition = Rotations.zero();
        public Voltage setPivotVoltage = Volts.zero();
        public Voltage setRollerVoltage = Volts.zero();
    }

    public default void zeroHere() {}
    public default void setPivotPosition(Angle position) {}
    public default void setRollerVoltage(Voltage voltage) {}

    public default void updateInputs(IntakeIOInputsAutoLogged inputs) {}
}
