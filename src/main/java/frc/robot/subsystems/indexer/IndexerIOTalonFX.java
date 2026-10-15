package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.motorstatus.TalonFXStatus;

public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX motor;
  private final TalonFXStatus motorStatus;
  private final VoltageOut voltageRequest;

  public IndexerIOTalonFX() {
    this.motor = new TalonFX(IndexerConstants.kMotorId);
    this.motorStatus = new TalonFXStatus(motor);
    this.voltageRequest = new VoltageOut(0);

    motor.getConfigurator().apply(IndexerConstants.Configs.TalonFXConfiguration);
    motorStatus.setUpdateFrequency(IndexerConstants.kStatusFrequency);
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    motor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void updateInputs(IndexerIOInputsAutoLogged inputs) {
    motorStatus.refresh();

    inputs.connected = motorStatus.getConnected();
    inputs.velocity = motorStatus.getVelocity();
    inputs.appliedVoltage = motorStatus.getAppliedVoltage();
    inputs.supplyCurrent = motorStatus.getSupplyCurrent();
    inputs.statorCurrent = motorStatus.getStatorCurrent();
    inputs.temperature = motorStatus.getTemperature();
  }
}
