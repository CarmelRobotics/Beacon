package frc.robot.subsystems.shooter.kicker;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.SimMotorStatus;

public class ShooterKickerIOSim implements ShooterKickerIO {
    private final DCMotorSim motor;
    private final SimMotorStatus motorStatus;

    public ShooterKickerIOSim() {
        this.motor =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    ShooterConstants.KickerConfigs.kSimMotor,
                    ShooterConstants.KickerConfigs.kSimMotorMOIJ,
                    ShooterConstants.KickerConfigs.kSimMotorGearRatio
                    ),
                ShooterConstants.KickerConfigs.kSimMotor
        );

        this.motorStatus = new SimMotorStatus(motor);
    }

    @Override
    public void updatePID(double kA, double kD, double kI, double kP, double kS, double kV) {}
    
    @Override
    public void setVelocity(AngularVelocity velocity) {
        motor.setAngularVelocity(velocity.in(RadiansPerSecond));
    }

    @Override
    public void setOutput(double output) {
        motor.setInputVoltage(12.0 * output);
    }
    
    @Override
    public void updateInputs(ShooterKickerIOInputsAutoLogged inputs) {
        inputs.connected = motorStatus.getConnected();
        inputs.velocity = motorStatus.getVelocity();
        inputs.appliedVoltage = motorStatus.getAppliedVoltage();
        inputs.supplyCurrent = motorStatus.getSupplyCurrent();
        inputs.statorCurrent = motorStatus.getStatorCurrent();
        inputs.temperature = motorStatus.getTemperature();
    }
}
