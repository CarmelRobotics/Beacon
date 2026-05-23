package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.SimMotorStatus;

public class IndexerIOSim implements IndexerIO {
    private final DCMotorSim indexerMotor;
    private final SimMotorStatus indexerStatus;

    public IndexerIOSim() {
        this.indexerMotor =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    IndexerConstants.Configs.kSimMotor,
                    IndexerConstants.Configs.kSimMotorMOIJ,
                    IndexerConstants.Configs.kSimMotorGearRatio
                    ),
                IndexerConstants.Configs.kSimMotor
        );

        this.indexerStatus = new SimMotorStatus(indexerMotor);
    }

    @Override
    public void setIndexerVoltage(Voltage voltage) {
        indexerMotor.setInputVoltage(voltage.in(Volts));
    }

    @Override
    public void updateInputs(IndexerIOInputsAutoLogged inputs) {
        inputs.connected = indexerStatus.getConnected();
        inputs.velocity = indexerStatus.getVelocity();
        inputs.appliedVoltage = indexerStatus.getAppliedVoltage();
        inputs.supplyCurrent = indexerStatus.getSupplyCurrent();
        inputs.statorCurrent = indexerStatus.getStatorCurrent();
        inputs.temperature = indexerStatus.getTemperature();
    }
}
