// Copyright (c) 2025-2026 Rockin' Bots
// https://github.com/CarmelRobotics

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
    public boolean leaderConnected;
    public Angle leaderPosition;
    public AngularVelocity leaderVelocity;
    public double leaderOutput;
    public Voltage leaderAppliedVoltage;
    public Current leaderSupplyCurrent;
    public Current leaderStatorCurrent;
    public Temperature leaderTemperature;

    public boolean followerConnected;
    public Angle followerPosition;
    public AngularVelocity followerVelocity;
    public double followerOutput;
    public Voltage followerAppliedVoltage;
    public Current followerSupplyCurrent;
    public Current followerStatorCurrent;
    public Temperature followerTemperature;
  }

  public default void zeroHere() {};

  public default void updatePID(double kA, double kD, double kI, double kP, double kS, double kV) {}

  public default void setPosition(Angle position) {}

  public default void updateInputs(IntakePivotIOInputsAutoLogged inputs) {}
}
