package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIO.IndexerState;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs;
    @AutoLogOutput(key = "Indexer/IndexerState")
    public IndexerState indexerState;
    private final Alert indexerMotorDisconnected;

    public Indexer(IndexerIO io) {
        this.io = io;
        this.inputs = new IndexerIOInputsAutoLogged();
        this.indexerState = IndexerState.OFF;
        this.indexerMotorDisconnected = new Alert("[Indexer] Indexer Motor Disconnected.", AlertType.kError);
    }

    public void setState(IndexerState indexerState) {
        this.indexerState = indexerState;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.setIndexerVoltage(indexerState.voltage);
        indexerMotorDisconnected.set(!inputs.connected);
    }
}
