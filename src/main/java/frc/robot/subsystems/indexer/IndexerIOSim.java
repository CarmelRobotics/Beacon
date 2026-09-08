package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.motorstatus.SimMotorStatus;

public class IndexerIOSim implements IndexerIO {
  private final DCMotorSim motor;
  private final SimMotorStatus motorStatus;

  public IndexerIOSim() {
    this.motor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            IndexerConstants.Configs.kSimMotor,
            IndexerConstants.Configs.kSimMotorMOIJ,
            IndexerConstants.Configs.kSimMotorGearRatio),
        IndexerConstants.Configs.kSimMotor);

    this.motorStatus = new SimMotorStatus(motor);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    motor.setInputVoltage(voltage.in(Volts));
  }

  @Override
  public void updateInputs(IndexerIOInputsAutoLogged inputs) {
    inputs.connected = motorStatus.getConnected();
    inputs.velocity = motorStatus.getVelocity();
    inputs.appliedVoltage = motorStatus.getAppliedVoltage();
    inputs.supplyCurrent = motorStatus.getSupplyCurrent();
    inputs.statorCurrent = motorStatus.getStatorCurrent();
    inputs.temperature = motorStatus.getTemperature();
  }
}
