package frc.robot.subsystems.shooter.kicker;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.TalonFXStatus;

public class ShooterKickerIOTalonFX implements ShooterKickerIO {
    private final TalonFX kickerMotor;
    private final TalonFXStatus kickerStatus;
    private final VelocityVoltage velocityRequest;
    private final VoltageOut voltageRequest;

    public ShooterKickerIOTalonFX() {
        this.kickerMotor = new TalonFX(ShooterConstants.kKickerMotorId);
        this.kickerStatus = new TalonFXStatus(kickerMotor);
        this.velocityRequest = new VelocityVoltage(0);
        this.voltageRequest = new VoltageOut(0);

        kickerMotor.getConfigurator().apply(ShooterConstants.KickerConfigs.TalonFXConfiguration);

        kickerStatus.setUpdateFrequency(ShooterConstants.kStatusUpdateFrequency);
    }

    @Override
    public void updatePID(double kA, double kD, double kI, double kP, double kS, double kV) {
        kickerMotor.getConfigurator().apply(new Slot0Configs()
            .withKA(kA).withKD(kD).withKI(kI).withKP(kP).withKS(kS).withKV(kV)
        );
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        kickerMotor.setControl(velocityRequest.withVelocity(velocity));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        kickerMotor.setControl(voltageRequest.withOutput(voltage));
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
