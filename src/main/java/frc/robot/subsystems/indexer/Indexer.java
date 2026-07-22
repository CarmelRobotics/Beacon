package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public Command setState(IndexerState indexerState) {
        switch (indexerState) {
            case IDLE:
                return run(() -> io.setVoltage(IndexerConstants.kIdleVoltage));
            case FEED:
                return run(() -> io.setVoltage(IndexerConstants.kFeedVoltage));
            case EJECT:
                return run(() -> io.setVoltage(IndexerConstants.kEjectVoltage));
        }

        return Commands.none();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        motorDisconnected.set(!inputs.connected);
    }
}
