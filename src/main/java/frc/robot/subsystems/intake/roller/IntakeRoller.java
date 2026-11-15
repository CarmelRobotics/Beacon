// Copyright (c) 2025-2026 Rockin' Bots
// https://github.com/CarmelRobotics

package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRollerState;

public class IntakeRoller extends SubsystemBase {
  private final IntakeRollerIO io;
  private final IntakeRollerIOInputsAutoLogged inputs;

  private final BooleanSupplier pivotDeployed;

  private final Alert leaderDisconnected;
  private final Alert followerDisconnected;

  @AutoLogOutput(key = "Intake/Roller/Voltage")
  private Voltage targetVoltage;
  @AutoLogOutput(key = "Intake/Roller/State")
  private IntakeRollerState state;

  public IntakeRoller(IntakeRollerIO io, BooleanSupplier pivotDeployed) {
    this.io = io;
    this.inputs = new IntakeRollerIOInputsAutoLogged();

    this.pivotDeployed = pivotDeployed;

    this.leaderDisconnected = new Alert("[Intake/Roller] Leader Roller Motor Disconnected.", AlertType.kError);
    this.followerDisconnected = new Alert("[Intake/Roller] Follower Roller Motor Disconnected.", AlertType.kError);

    this.targetVoltage = Volts.zero();
    this.state = IntakeRollerState.kIdle;
  }

  public Command setState(IntakeRollerState state) {
    return (switch (state) {
      case kIdle -> runOnce(() -> setVoltage(IntakeConstants.kIdleVoltage));
      case kIntake ->
        Commands.waitUntil(pivotDeployed)
            .andThen(runOnce(() -> setVoltage(IntakeConstants.kIntakeVoltage)));
      case kEject ->
        Commands.waitUntil(pivotDeployed)
            .andThen(runOnce(() -> setVoltage(IntakeConstants.kEjectVoltage)));
    }).beforeStarting(() -> { this.state = state; })
      .withName("[Intake/Roller] Run from state: " + state.toString());
  }

  public void setIdle() {
    this.state = IntakeRollerState.kIdle;
    setVoltage(IntakeConstants.kIdleVoltage);
  }

  public IntakeRollerState getState() {
    return state;
  }

  private void setVoltage(Voltage voltage) {
    targetVoltage = voltage;
    io.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Roller/Inputs", inputs);
    leaderDisconnected.set(!inputs.leaderConnected);
    followerDisconnected.set(!inputs.followerConnected);
  }
}
