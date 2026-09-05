package frc.robot.subsystems.shooter.roller;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.motorstatus.TalonFXStatus;

public class ShooterRollerIOTalonFX implements ShooterRollerIO {
    private final TalonFX frontLeftLeaderMotor;
    private final TalonFX backLeftFollowerMotor;
    private final TalonFX frontRightFollowerMotor;
    private final TalonFX backRightFollowerMotor;

    private final TalonFXStatus frontLeftLeaderStatus;
    private final TalonFXStatus backLeftFollowerStatus;
    private final TalonFXStatus frontRightFollowerStatus;
    private final TalonFXStatus backRightFollowerStatus;

    private final VelocityVoltage velocityRequest;
    private final DutyCycleOut dutyCycyleRequest;

    public ShooterRollerIOTalonFX() {
        this.frontLeftLeaderMotor = new TalonFX(ShooterConstants.kFrontLeftLeaderRollerMotorId);
        this.backLeftFollowerMotor = new TalonFX(ShooterConstants.kBackLeftFollowerRollerMotorId);
        this.frontRightFollowerMotor = new TalonFX(ShooterConstants.kFrontRightFollowerRollerMotorId);
        this.backRightFollowerMotor = new TalonFX(ShooterConstants.kBackRightFollowerRollerMotorId);

        this.frontLeftLeaderStatus = new TalonFXStatus(frontLeftLeaderMotor);
        this.backLeftFollowerStatus = new TalonFXStatus(backLeftFollowerMotor);
        this.frontRightFollowerStatus = new TalonFXStatus(frontRightFollowerMotor);
        this.backRightFollowerStatus = new TalonFXStatus(backRightFollowerMotor);

        this.velocityRequest = new VelocityVoltage(0);
        this.dutyCycyleRequest = new DutyCycleOut(0);

        frontLeftLeaderMotor.getConfigurator().apply(ShooterConstants.RollerConfigs.TalonFXConfiguration);

        backLeftFollowerMotor.setControl(new Follower(frontLeftLeaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        frontRightFollowerMotor.setControl(new Follower(frontLeftLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        backRightFollowerMotor.setControl(new Follower(frontLeftLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        frontLeftLeaderStatus.setUpdateFrequency(ShooterConstants.kStatusUpdateFrequency);
        backLeftFollowerStatus.setUpdateFrequency(ShooterConstants.kStatusUpdateFrequency);
        frontRightFollowerStatus.setUpdateFrequency(ShooterConstants.kStatusUpdateFrequency);
        backRightFollowerStatus.setUpdateFrequency(ShooterConstants.kStatusUpdateFrequency);
    }

    @Override
    public void updatePID(double kA, double kD, double kI, double kP, double kS, double kV) {
        final Slot0Configs slot0configs = new Slot0Configs()
            .withKA(kA).withKD(kD).withKI(kI).withKP(kP).withKS(kS).withKV(kV);

        frontLeftLeaderMotor.getConfigurator().apply(slot0configs);
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        frontLeftLeaderMotor.setControl(velocityRequest.withVelocity(velocity));
    }

    @Override
    public void setOutput(double output) {
        frontLeftLeaderMotor.setControl(dutyCycyleRequest.withOutput(output));
    }

    @Override
    public void updateInputs(ShooterRollerIOInputsAutoLogged inputs) {
        inputs.frontLeftConnected = frontLeftLeaderStatus.getConnected();
        inputs.frontLeftVelocity = frontLeftLeaderStatus.getVelocity();
        inputs.frontLeftAppliedVoltage = frontLeftLeaderStatus.getAppliedVoltage();
        inputs.frontLeftSupplyCurrent = frontLeftLeaderStatus.getSupplyCurrent();
        inputs.frontLeftStatorCurrent = frontLeftLeaderStatus.getStatorCurrent();
        inputs.frontLeftTemperature = frontLeftLeaderStatus.getTemperature();

        inputs.backLeftConnected = backLeftFollowerStatus.getConnected();
        inputs.backLeftVelocity = backLeftFollowerStatus.getVelocity();
        inputs.backLeftAppliedVoltage = backLeftFollowerStatus.getAppliedVoltage();
        inputs.backLeftSupplyCurrent = backLeftFollowerStatus.getSupplyCurrent();
        inputs.backLeftStatorCurrent = backLeftFollowerStatus.getStatorCurrent();
        inputs.backLeftTemperature = backLeftFollowerStatus.getTemperature();

        inputs.frontRightConnected = frontRightFollowerStatus.getConnected();
        inputs.frontRightVelocity = frontRightFollowerStatus.getVelocity();
        inputs.frontRightAppliedVoltage = frontRightFollowerStatus.getAppliedVoltage();
        inputs.frontRightSupplyCurrent = frontRightFollowerStatus.getSupplyCurrent();
        inputs.frontRightStatorCurrent = frontRightFollowerStatus.getStatorCurrent();
        inputs.frontRightTemperature = frontRightFollowerStatus.getTemperature();

        inputs.backRightConnected = backRightFollowerStatus.getConnected();
        inputs.backRightVelocity = backRightFollowerStatus.getVelocity();
        inputs.backRightAppliedVoltage = backRightFollowerStatus.getAppliedVoltage();
        inputs.backRightSupplyCurrent = backRightFollowerStatus.getSupplyCurrent();
        inputs.backRightStatorCurrent = backRightFollowerStatus.getStatorCurrent();
        inputs.backRightTemperature = backRightFollowerStatus.getTemperature();
    }
}
