package frc.robot.subsystems.shooter.kicker;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.SimMotorStatus;

public class ShooterKickerIOSim implements ShooterKickerIO {
    private final DCMotorSim kickerMotor;
    private final SimMotorStatus kickerStatus;

    public ShooterKickerIOSim() {
        this.kickerMotor =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    ShooterConstants.KickerConfigs.kSimMotor,
                    ShooterConstants.KickerConfigs.kSimMotorMOIJ,
                    ShooterConstants.KickerConfigs.kSimMotorGearRatio
                    ),
                ShooterConstants.KickerConfigs.kSimMotor
        );

        this.kickerStatus = new SimMotorStatus(kickerMotor);
    }

    @Override
    public void updatePID(double kA, double kD, double kI, double kP, double kS, double kV) {}
    
    @Override
    public void setVelocity(AngularVelocity velocity) {
        kickerMotor.setAngularVelocity(velocity.in(RadiansPerSecond));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        kickerMotor.setInputVoltage(voltage.in(Volts));
    }
    
    @Override
    public void updateInputs(ShooterKickerIOInputsAutoLogged inputs) {
        inputs.connected = kickerStatus.getConnected();
        inputs.velocity = kickerStatus.getVelocity();
        inputs.appliedVoltage = kickerStatus.getAppliedVoltage();
        inputs.supplyCurrent = kickerStatus.getSupplyCurrent();
        inputs.statorCurrent = kickerStatus.getStatorCurrent();
        inputs.temperature = kickerStatus.getTemperature();
    }
}
