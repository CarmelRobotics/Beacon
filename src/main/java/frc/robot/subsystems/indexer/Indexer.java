package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerState;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs;

  private final Alert motorDisconnected;

  public Indexer(IndexerIO io) {
    this.io = io;
    this.inputs = new IndexerIOInputsAutoLogged();

    this.motorDisconnected = new Alert("[Indexer] Indexer Motor Disconnected.", AlertType.kError);
  }

  public Command setState(IndexerState state) {
    return (switch (state) {
      case kIdle -> run(() -> io.setVoltage(IndexerConstants.kIdleVoltage));
      case kFeed -> run(() -> io.setVoltage(IndexerConstants.kFeedVoltage));
      case kEject -> run(() -> io.setVoltage(IndexerConstants.kEjectVoltage));
    }).withName("[Indexer] Run from state: " + state.toString());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    motorDisconnected.set(!inputs.connected);
  }
}
