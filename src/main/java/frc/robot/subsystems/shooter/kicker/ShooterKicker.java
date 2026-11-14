package frc.robot.subsystems.shooter.kicker;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterKickerState;

public class ShooterKicker extends SubsystemBase {
  private final ShooterKickerIO io;
  private final ShooterKickerIOInputsAutoLogged inputs;

  private final Alert motorDisconnected;

  private final BooleanSupplier readyToFeed;

  @AutoLogOutput(key = "Shooter/Kicker/Velocity")
  private AngularVelocity targetVelocity;
  @AutoLogOutput(key = "Shooter/Kicker/Voltage")
  private Voltage targetVoltage;
  @AutoLogOutput(key = "Shooter/Kicker/Output")
  private double targetOutput;
  @AutoLogOutput(key = "Shooter/Kicker/State")
  private ShooterKickerState state;

  public ShooterKicker(
    ShooterKickerIO io,
    BooleanSupplier readyToFeed) {
    this.io = io;
    this.inputs = new ShooterKickerIOInputsAutoLogged();

    this.motorDisconnected = new Alert("[Shooter/Kicker] Kicker Motor Disconnected.", AlertType.kError);

    this.readyToFeed = readyToFeed;

    this.targetVelocity = RotationsPerSecond.zero();
    this.targetOutput = 0.0;
    this.state = ShooterKickerState.kIdle;
  }

  public Command setState(ShooterKickerState state) {
    return (switch (state) {
      case kIdle -> runOnce(() -> setVoltage(ShooterConstants.kKickerIdleVoltage));
      case kEject -> runOnce(() -> setVoltage(ShooterConstants.kKickerEjectVoltage));
      case kFeed ->
        Commands.waitTime(Constants.kRollerDelay)
          .andThen(runOnce(() -> setVoltage(ShooterConstants.kKickerFeedVoltage)));
    }).beforeStarting(() -> {this.state = state;})
      .withName("[Shooter/Kicker] Run from state: " + state.toString());
  }

  public ShooterKickerState getState() {
    return state;
  }

  public void setIdle() {
    this.state = ShooterKickerState.kIdle;
    setVoltage(ShooterConstants.kKickerIdleVoltage);
  }

  @SuppressWarnings("unused")
  private void setVelocity(AngularVelocity velocity) {
    targetVoltage = Volts.zero();
    targetOutput = 0.0;
    targetVelocity = velocity;
    io.setVelocity(velocity);
  }

  private void setVoltage(Voltage voltage) {
    targetVelocity = RotationsPerSecond.zero();
    targetOutput = 0.0;
    targetVoltage = voltage;
    io.setVoltage(voltage);
  }

  @SuppressWarnings("unused")
  private void setOutput(double output) {
    targetVoltage = Volts.zero();
    targetVelocity = RotationsPerSecond.zero();
    targetOutput = output;
    io.setOutput(output);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Kicker/Inputs", inputs);

    motorDisconnected.set(!inputs.connected);
  }
}
