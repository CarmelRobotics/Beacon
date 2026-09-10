package frc.robot.subsystems.shooter.roller;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  private final BooleanSupplier facingTarget;
  private final DoubleSupplier targetDistance;

  private final LoggedTunableNumber kA;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;

  @AutoLogOutput(key = "Shooter/Roller/Velocity")
  private AngularVelocity targetVelocity;
  @AutoLogOutput(key = "Shooter/Roller/Output")
  private double targetOutput;
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

    this.facingTarget = facingTarget;
    this.targetDistance = targetDistance;

    this.kA = new LoggedTunableNumber("Shooter/Roller/kA", ShooterConstants.PivotConfigs.kA);
    this.kD = new LoggedTunableNumber("Shooter/Roller/kD", ShooterConstants.PivotConfigs.kD);
    this.kI = new LoggedTunableNumber("Shooter/Roller/kI", ShooterConstants.PivotConfigs.kI);
    this.kP = new LoggedTunableNumber("Shooter/Roller/kP", ShooterConstants.PivotConfigs.kP);
    this.kS = new LoggedTunableNumber("Shooter/Roller/kS", ShooterConstants.PivotConfigs.kS);
    this.kV = new LoggedTunableNumber("Shooter/Roller/kV", ShooterConstants.PivotConfigs.kV);

    this.targetVelocity = RotationsPerSecond.zero();
    this.targetOutput = 0.0;
    this.state = ShooterRollerState.kIdle;
  }

  public Command setState(ShooterRollerState state) {
    return (switch (state) {
      case kIdle -> runOnce(() -> setVelocity(ShooterConstants.kRollerIdleVelocity));
      case kStaticScore -> runOnce(() -> setVelocity(ShooterConstants.kRollerStaticScoreVelocity));
      case kStaticSend -> runOnce(() -> setOutput(ShooterConstants.kRollerStaticSendOutput));
      case kPass -> 
        Commands.waitUntil(facingTarget)
          .andThen(runOnce(() -> setVelocity(RotationsPerSecond.of(
              ShooterConstants.RollerPassingMap.get(targetDistance.getAsDouble())))));
      case kScore -> 
        Commands.waitUntil(facingTarget)
          .andThen(runOnce(() -> setVelocity(RotationsPerSecond.of(
              ShooterConstants.RollerScoringMap.get(targetDistance.getAsDouble())))));
    }).beforeStarting(() -> {this.state = state;})
      .withName("[Shooter/Roller] Run from state: " + state.toString());
  }

  public AngularVelocity getTargetVelocity() {
    return targetVelocity;
  }

  public boolean isReadyToScore() {
    return inputs.reachedTarget && facingTarget.getAsBoolean();
  }

  public ShooterRollerState getState() {
    return state;
  }

  private void setVelocity(AngularVelocity velocity) {
    targetOutput = 0.0;
    targetVelocity = velocity;
    io.setVelocity(velocity);
  }

  private void setOutput(double output) {
    targetVelocity = RotationsPerSecond.zero();
    targetOutput = output;
    io.setOutput(output);
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
