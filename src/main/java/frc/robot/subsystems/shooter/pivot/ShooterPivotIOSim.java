// Copyright (c) 2025-2026 Rockin' Bots
// https://github.com/CarmelRobotics

package frc.robot.subsystems.shooter.pivot;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.motorstatus.SimMotorStatus;

public class ShooterPivotIOSim implements ShooterPivotIO {
  private final DCMotorSim motor;
  private final SimMotorStatus motorStatus;

  public ShooterPivotIOSim() {
    this.motor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            ShooterConstants.PivotConfigs.kSimMotor,
            ShooterConstants.PivotConfigs.kSimMotorMOIJ,
            ShooterConstants.PivotConfigs.kSimMotorGearRatio),
        ShooterConstants.PivotConfigs.kSimMotor);

    this.motorStatus = new SimMotorStatus(motor);
  }

  @Override
  public void setPosition(Angle position) {
    motor.setAngle(position.in(Radians));
  }

  @Override
  public void updateInputs(ShooterPivotIOInputsAutoLogged inputs) {
    inputs.connected = motorStatus.getConnected();
    inputs.position = motorStatus.getPosition();
    inputs.velocity = motorStatus.getVelocity();
    inputs.output = motorStatus.getOutput();
    inputs.appliedVoltage = motorStatus.getAppliedVoltage();
    inputs.statorCurrent = motorStatus.getSupplyCurrent();
    inputs.temperature = motorStatus.getTemperature();
  }
}
