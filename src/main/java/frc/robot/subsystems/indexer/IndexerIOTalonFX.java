package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.TalonFXStatus;

public class IndexerIOTalonFX implements IndexerIO {
    private final TalonFX indexerMotor;
    private final TalonFXStatus indexerStatus;
    private final VoltageOut voltageRequest;
    
    public IndexerIOTalonFX() {
        this.indexerMotor = new TalonFX(IndexerConstants.kMotorId);
        this.indexerStatus = new TalonFXStatus(indexerMotor);
        this.voltageRequest = new VoltageOut(0);

        indexerMotor.getConfigurator().apply(IndexerConstants.Configs.TalonFXConfiguration);
        indexerStatus.setUpdateFrequency(IndexerConstants.kStatusUpdateFrequency);
    }

    @Override
    public void setIndexerVoltage(Voltage voltage) {
        indexerMotor.setControl(voltageRequest.withOutput(voltage));
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
