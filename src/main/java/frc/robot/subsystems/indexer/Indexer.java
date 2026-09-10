package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerState;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs;

  private final Alert motorDisconnected;

  @AutoLogOutput
  private Voltage targetVoltage;

  public Indexer(IndexerIO io) {
    this.io = io;
    this.inputs = new IndexerIOInputsAutoLogged();

    this.motorDisconnected = new Alert("[Indexer] Indexer Motor Disconnected.", AlertType.kError);

    this.targetVoltage = Volts.zero();
  }

  public Command setState(IndexerState state) {
    return (switch (state) {
      case kIdle -> run(() -> setVoltage(IndexerConstants.kIdleVoltage));
      case kFeed -> run(() -> setVoltage(IndexerConstants.kFeedVoltage));
      case kEject -> run(() -> setVoltage(IndexerConstants.kEjectVoltage));
    }).withName("[Indexer] Run from state: " + state.toString());
  }

  private void setVoltage(Voltage voltage) {
    targetVoltage = voltage;
    io.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    motorDisconnected.set(!inputs.connected);
  }
}
