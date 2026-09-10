// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
    public boolean driveConnected = false;
    public Angle drivePosition = Radians.zero();
    public AngularVelocity driveVelocity = RadiansPerSecond.zero();
    public Voltage driveAppliedVoltage = Volts.zero();
    public Current driveSupplyCurrent = Amps.zero();
    public Current driveStatorCurrent = Amps.zero();
    public Temperature driveTemperature = Celsius.zero();

    public boolean steerConnected = false;
    public Rotation2d steerPosition = new Rotation2d();
    public AngularVelocity steerVelocity = RadiansPerSecond.zero();
    public Voltage steerAppliedVoltage = Volts.zero();
    public Current steerSupplyCurrent = Amps.zero();
    public Current steerStatorCurrent = Amps.zero();
    public Temperature steerTemperature = Celsius.zero();

    public boolean encoderConnected = false;
    public boolean encoderMagnetIsOk = false;
    public Rotation2d encoderPosition = new Rotation2d();

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRads = new double[] {};
    public Rotation2d[] odometrySteerPositions = new Rotation2d[] {};
  }

  public default void setDriveVoltage(Voltage voltage) {}

  public default void setSteerVoltage(Voltage voltage) {}

  public default void setVelocity(AngularVelocity velocity) {}

  public default void setSteer(Rotation2d rotation) {}

  public default void updateInputs(ModuleIOInputsAutoLogged inputs) {}
}
