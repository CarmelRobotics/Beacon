package frc.robot.subsystems.shooter.pivot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.TalonFXStatus;

public class ShooterPivotIOTalonFX implements ShooterPivotIO {
    private final TalonFX pivotMotor;
    private final TalonFXStatus pivotStatus;
    private final MotionMagicVoltage positionRequest;

    public ShooterPivotIOTalonFX() {
        this.pivotMotor = new TalonFX(ShooterConstants.kPivotMotorId);
        this.pivotStatus = new TalonFXStatus(pivotMotor);
        this.positionRequest = new MotionMagicVoltage(0);

        pivotMotor.getConfigurator().apply(ShooterConstants.PivotConfigs.TalonFXConfiguration);
        pivotStatus.setUpdateFrequency(ShooterConstants.kStatusUpdateFrequency);
    }

    @Override
    public void zeroPivotHere() {
        pivotMotor.setPosition(0);
    }

    @Override
    public void updatePivotPID(double kA, double kD, double kI, double kP, double kS, double kV) {
        pivotMotor.getConfigurator().apply(new Slot0Configs()
            .withKA(kA).withKD(kD).withKI(kI).withKP(kP).withKS(kS).withKV(kV)
        );
    }

    @Override
    public void updatePivotMotionMagic(double kCruiseVelocity, double kAcceleration, double kJerk) {
        pivotMotor.getConfigurator().apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(kCruiseVelocity)
            .withMotionMagicAcceleration(kAcceleration)
            .withMotionMagicJerk(kJerk)
        );
    }

    @Override
    public void setPivotPosition(Angle position) {
        pivotMotor.setControl(positionRequest.withPosition(position));
    }

    @Override
    public void updateInputs(ShooterPivotIOInputsAutoLogged inputs) {
        inputs.connected = pivotStatus.getConnected();
        inputs.position = pivotStatus.getPosition();
        inputs.velocity = pivotStatus.getVelocity();
        inputs.appliedVoltage = pivotStatus.getAppliedVoltage();
        inputs.statorCurrent = pivotStatus.getSupplyCurrent();
        inputs.temperature = pivotStatus.getTemperature();
    }
}
