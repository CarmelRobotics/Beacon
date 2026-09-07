// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

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
        public Angle drivePosition;
        public AngularVelocity driveVelocity;
        public Voltage driveAppliedVoltage;
        public Current driveSupplyCurrent;
        public Current driveStatorCurrent;
        public Temperature driveTemperature;

        public boolean steerConnected;
        public Rotation2d steerPosition;
        public AngularVelocity steerVelocity;
        public Voltage steerAppliedVoltage;
        public Current steerSupplyCurrent;
        public Current steerStatorCurrent;
        public Temperature steerTemperature;

        public boolean encoderConnected;
        public boolean encoderMagnetIsOk;
        public Rotation2d encoderPosition;

        public double[] odometryTimestamps;
        public double[] odometryDrivePositionsRads;
        public Rotation2d[] odometrySteerPositions;
    }

    public void setDriveVoltage(Voltage voltage);
    public void setSteerVoltage(Voltage voltage);
    public void setVelocity(AngularVelocity velocity);
    public void setSteer(Rotation2d rotation);
    public void updateInputs(ModuleIOInputsAutoLogged inputs);
}
