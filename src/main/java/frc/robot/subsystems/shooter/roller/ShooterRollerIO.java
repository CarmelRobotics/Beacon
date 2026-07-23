package frc.robot.subsystems.shooter.roller;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface ShooterRollerIO {
    @AutoLog
    public static class ShooterRollerIOInputs {
        public boolean frontLeftConnected;
        public AngularVelocity frontLeftVelocity;
        public Voltage frontLeftAppliedVoltage;
        public Current frontLeftSupplyCurrent;
        public Current frontLeftStatorCurrent;
        public Temperature frontLeftTemperature;

        public boolean backLeftConnected;
        public AngularVelocity backLeftVelocity;
        public Voltage backLeftAppliedVoltage;
        public Current backLeftSupplyCurrent;
        public Current backLeftStatorCurrent;
        public Temperature backLeftTemperature;

        public boolean frontRightConnected;
        public AngularVelocity frontRightVelocity;
        public Voltage frontRightAppliedVoltage;
        public Current frontRightSupplyCurrent;
        public Current frontRightStatorCurrent;
        public Temperature frontRightTemperature;

        public boolean backRightConnected;
        public AngularVelocity backRightVelocity;
        public Voltage backRightAppliedVoltage;
        public Current backRightSupplyCurrent;
        public Current backRightStatorCurrent;
        public Temperature backRightTemperature;
    }

    public void updatePID(double kA, double kD, double kI, double kP, double kS, double kV);
    public void setVelocity(AngularVelocity velocity);
    public void setOutput(double output);
    public void updateInputs(ShooterRollerIOInputsAutoLogged inputs);
}
