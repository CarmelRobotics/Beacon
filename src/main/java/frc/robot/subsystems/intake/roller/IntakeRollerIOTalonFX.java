package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.TalonFXStatus;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
    private final TalonFX motor;
    private final TalonFXStatus motorStatus;

    private final VoltageOut voltageRequest;

    public IntakeRollerIOTalonFX() {
        this.motor = new TalonFX(IntakeConstants.kRollerMotorId);
        this.motorStatus = new TalonFXStatus(motor);
        this.voltageRequest = new VoltageOut(0);

        motor.getConfigurator().apply(IntakeConstants.RollerConfigs.TalonFXConfiguration);
        motorStatus.setUpdateFrequency(IntakeConstants.kStatusUpdateFrequency);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        inputs.connected = motorStatus.getConnected();
        inputs.velocity = motorStatus.getVelocity();
        inputs.appliedVoltage = motorStatus.getAppliedVoltage();
        inputs.supplyCurrent = motorStatus.getSupplyCurrent();
        inputs.statorCurrent = motorStatus.getStatorCurrent();
        inputs.temperature = motorStatus.getTemperature();
    }
}
