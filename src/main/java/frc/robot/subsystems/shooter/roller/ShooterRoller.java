// Copyright (c) 2025-2026 Rockin' Bots
// https://github.com/CarmelRobotics

package frc.robot.subsystems.shooter.roller;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterRollerState;
import frc.robot.util.LoggedTunableNumber;

public class ShooterRoller extends SubsystemBase {
  private final ShooterRollerIO io;
  private final ShooterRollerIOInputsAutoLogged inputs;

  private final Alert frontLeftMotorDisconnected;
  private final Alert backLeftMotorDisconnected;
  private final Alert frontRightMotorDisconnected;
  private final Alert backRightMotorDisconnected;

  private final LoggedTunableNumber kA;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;

  private final BooleanSupplier facingTarget;
  private final DoubleSupplier targetDistance;

  @AutoLogOutput(key = "Shooter/Roller/Velocity")
  private AngularVelocity targetVelocity;
  @AutoLogOutput(key = "Shooter/Roller/Output")
  private double targetOutput;
  @AutoLogOutput(key = "Shooter/Roller/Voltage")
  private Voltage targetVoltage;
  @AutoLogOutput(key = "Shooter/Roller/State")
  private ShooterRollerState state;

  public ShooterRoller(ShooterRollerIO io, BooleanSupplier facingTarget, DoubleSupplier targetDistance) {
    this.io = io;
    this.inputs = new ShooterRollerIOInputsAutoLogged();

    this.frontLeftMotorDisconnected = new Alert("[Shooter/Roller] Front Left Roller Motor Disconnected.",
        AlertType.kError);
    this.backLeftMotorDisconnected = new Alert("[Shooter/Roller] Back Left Roller Motor Disconnected.",
        AlertType.kError);
    this.frontRightMotorDisconnected = new Alert("[Shooter/Roller] Front Right Roller Motor Disconnected.",
        AlertType.kError);
    this.backRightMotorDisconnected = new Alert("[Shooter/Roller] Back Right Roller Motor Disconnected.",
        AlertType.kError);

    this.kA = new LoggedTunableNumber("Shooter/Roller/kA", ShooterConstants.RollerConfigs.kA);
    this.kD = new LoggedTunableNumber("Shooter/Roller/kD", ShooterConstants.RollerConfigs.kD);
    this.kI = new LoggedTunableNumber("Shooter/Roller/kI", ShooterConstants.RollerConfigs.kI);
    this.kP = new LoggedTunableNumber("Shooter/Roller/kP", ShooterConstants.RollerConfigs.kP);
    this.kS = new LoggedTunableNumber("Shooter/Roller/kS", ShooterConstants.RollerConfigs.kS);
    this.kV = new LoggedTunableNumber("Shooter/Roller/kV", ShooterConstants.RollerConfigs.kV);

    this.facingTarget = facingTarget;
    this.targetDistance = targetDistance;

    this.targetVelocity = RotationsPerSecond.zero();
    this.targetOutput = 0.0;
    this.targetVoltage = Volts.zero();
    this.state = ShooterRollerState.kIdle;
  }

  public Command setState(ShooterRollerState state) {
    return (switch (state) {
      case kIdle -> runOnce(() -> setVoltage(ShooterConstants.kRollerIdleVoltage));
      case kPass ->
        run(() -> setVelocity(ShooterConstants.kRollerPassVelocity));
      case kStaticScore ->
        run(() -> setVelocity(ShooterConstants.kRollerScoreVelocity));
      case kScore ->
        run(() -> setVelocity(
            RotationsPerSecond.of(ShooterConstants.RollerScoringVelocityMap.get(targetDistance.getAsDouble()))));
    }).beforeStarting(() -> { this.state = state; })
      .withName("[Shooter/Roller] Run from state: " + state.toString());
  }

  public AngularVelocity getTargetVelocity() {
    return targetVelocity;
  }

  @AutoLogOutput(key = "Shooter/Roller/ReadyToFeed")
  public boolean isReadyToFeed() {
    return inputs.frontLeftVelocity.isNear(targetVelocity, ShooterConstants.kRollerToleranceVelocity)
        && facingTarget.getAsBoolean();
  }

  public ShooterRollerState getState() {
    return state;
  }

  public void setIdle() {
    this.state = ShooterRollerState.kIdle;
    setVoltage(Volts.zero());
  }

  private void setVelocity(AngularVelocity velocity) {
    targetOutput = 0.0;
    targetVoltage = Volts.zero();
    targetVelocity = velocity;
    io.setVelocity(velocity);
  }

  @SuppressWarnings("unused")
  private void setOutput(double output) {
    targetVelocity = RotationsPerSecond.zero();
    targetVoltage = Volts.zero();
    targetOutput = output;
    io.setOutput(output);
  }

  private void setVoltage(Voltage voltage) {
    targetOutput = 0.0;
    targetVelocity = RotationsPerSecond.zero();
    io.setVoltage(voltage);
  }

  private boolean hasPIDChanged() {
    return kA.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kI.hasChanged(hashCode())
        || kP.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Roller/Inputs", inputs);

    if (hasPIDChanged()) {
      io.updatePID(
          kA.get(),
          kD.get(),
          kI.get(),
          kP.get(),
          kS.get(),
          kV.get());
    }

    frontLeftMotorDisconnected.set(!inputs.frontLeftConnected);
    backLeftMotorDisconnected.set(!inputs.backLeftConnected);
    frontRightMotorDisconnected.set(!inputs.frontRightConnected);
    backRightMotorDisconnected.set(!inputs.backRightConnected);
  }
}
