package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.TalonFXStatus;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX pivotMotor;
    private final TalonFX rollerMotor;

    private final MotionMagicVoltage positionRequest;
    private final VoltageOut voltageRequest;

    private final TalonFXStatus pivotStatus;
    private final TalonFXStatus rollerStatus;

    public IntakeIOTalonFX() {
        this.pivotMotor = new TalonFX(IntakeConstants.kPivotMotorId);
        this.rollerMotor = new TalonFX(IntakeConstants.kRollerMotorId);

        this.pivotStatus = new TalonFXStatus(pivotMotor);
        this.rollerStatus = new TalonFXStatus(rollerMotor);

        this.positionRequest = new MotionMagicVoltage(0);
        this.voltageRequest = new VoltageOut(0);

        pivotMotor.getConfigurator().apply(IntakeConstants.PivotConfiguration);
        rollerMotor.getConfigurator().apply(IntakeConstants.RollerConfiguration);

        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        pivotStatus.setUpdateFrequency(IntakeConstants.kStatusUpdateFrequency);
        rollerStatus.setUpdateFrequency(IntakeConstants.kStatusUpdateFrequency);

        pivotMotor.optimizeBusUtilization();
        rollerMotor.optimizeBusUtilization();
    }

    @Override
    public void zeroHere() {
        pivotMotor.setPosition(0);
    }

    @Override
    public void setPivotPosition(Angle position) {
        pivotMotor.setControl(positionRequest.withPosition(position));
    }

    @Override
    public void setRollerVoltage(Voltage voltage) {
        rollerMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        inputs.pivotConnected = pivotMotor.isConnected();
        inputs.pivotPosition = pivotStatus.getPosition();
        inputs.pivotVelocity = pivotStatus.getVelocity();
        inputs.pivotAppliedVoltage = pivotStatus.getAppliedVoltage();
        inputs.pivotStatorCurrent = pivotStatus.getSupplyCurrent();
        inputs.pivotTemperature = pivotStatus.getTemperature();

        inputs.rollerConnected = rollerMotor.isConnected();
        inputs.rollerVelocity = rollerStatus.getVelocity();
        inputs.rollerAppliedVoltage = rollerStatus.getAppliedVoltage();
        inputs.rollerSupplyCurrent = rollerStatus.getSupplyCurrent();
        inputs.rollerStatorCurrent = rollerStatus.getStatorCurrent();
        inputs.rollerTemperature = rollerStatus.getTemperature();
    }
}
