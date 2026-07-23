package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.SimMotorStatus;

public class IntakeRollerIOSim implements IntakeRollerIO {
    private final DCMotorSim motor;
    private final SimMotorStatus motorStatus;

    public IntakeRollerIOSim() {
        this.motor =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    IntakeConstants.RollerConfigs.kSimMotor,
                    IntakeConstants.RollerConfigs.kSimMotorMOIJ,
                    IntakeConstants.RollerConfigs.kSimMotorGearRatio
                    ),
                IntakeConstants.RollerConfigs.kSimMotor
        );

        this.motorStatus = new SimMotorStatus(motor);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setInputVoltage(voltage.in(Volts));
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
