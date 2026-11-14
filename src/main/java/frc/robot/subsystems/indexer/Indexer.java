package frc.robot.subsystems.indexer;

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
import frc.robot.Constants;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerState;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs;
  private final BooleanSupplier readyToFeed;

  private final Alert motorDisconnected;

  @AutoLogOutput(key = "Indexer/Voltage")
  private Voltage targetVoltage;
  @AutoLogOutput(key = "Indexer/State")
  private IndexerState state;

  public Indexer(IndexerIO io, BooleanSupplier readyToFeed) {
    this.io = io;
    this.inputs = new IndexerIOInputsAutoLogged();
    this.readyToFeed = readyToFeed;

    this.motorDisconnected = new Alert("[Indexer] Indexer Motor Disconnected.", AlertType.kError);

    this.targetVoltage = Volts.zero();
    this.state = IndexerConstants.IndexerState.kIdle;
  }

  public Command setState(IndexerState state) {
    return (switch (state) {
      case kIdle -> runOnce(() -> setVoltage(IndexerConstants.kIdleVoltage));
      case kEject -> runOnce(() -> setVoltage(IndexerConstants.kEjectVoltage));
      case kFeed ->
        Commands.waitTime(Constants.kRollerDelay)
          .andThen(run(() -> setVoltage(IndexerConstants.kFeedVoltage)));
    }).beforeStarting(() -> {this.state = state;})
      .withName("[Indexer] Run from state: " + state.toString());
  }

  public void setIdle() {
    this.state = IndexerState.kIdle;
    setVoltage(IndexerConstants.kIdleVoltage);
  }

  public IndexerState getState() {
    return state;
  }

  private void setVoltage(Voltage voltage) {
    targetVoltage = voltage;
    io.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer/Inputs", inputs);
    motorDisconnected.set(!inputs.connected);
  }
}
