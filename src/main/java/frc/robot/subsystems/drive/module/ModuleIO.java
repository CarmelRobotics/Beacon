package frc.robot.subsystems.drive.module;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface ModuleIO {
    @AutoLog
    public class ModuleIOInputs {
        public boolean driveConnected;
        public AngularVelocity driveVelocity;
        public Voltage driveAppliedVoltage;
        public Current driveSupplyCurrent;
        public Current driveStatorCurrent;
        public Temperature driveTemperature;

        public boolean steerConnected;
        public Angle steerPosition;
        public AngularVelocity steerVelocity;
        public Voltage steerAppliedVoltage;
        public Current steerSupplyCurrent;
        public Current steerStatorCurrent;
        public Temperature steerTemperature;

        public boolean encoderConnected;
        public boolean encoderMagnentIsOk;
        public Angle encoderPosition;

        public double[] odometryTimestamps;
        public Angle[] odometryDrivePositions;
        public Rotation2d[] odometryTurnPositions;
    }

    public void setDriveOutput(double output);
    public void setSteerOutput(double output);
    public void setVelocity(AngularVelocity veloicty);
    public void setSteer(Rotation2d rotation);
    public void updateInputs(ModuleIOInputsAutoLogged inputs);
}
