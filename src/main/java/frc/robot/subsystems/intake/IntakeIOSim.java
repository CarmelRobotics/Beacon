package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.SimMotorStatus;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim pivotMotor;
    private final DCMotorSim rollerMotor;

    private final SimMotorStatus pivotStatus;
    private final SimMotorStatus rollerStatus;

    public IntakeIOSim() {
        this.pivotMotor = 
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    IntakeConstants.PivotConfigs.kSimMotor,
                    IntakeConstants.PivotConfigs.kSimMotorMOIJ,
                    IntakeConstants.PivotConfigs.kSimMotorGearRatio
                    ),
                IntakeConstants.PivotConfigs.kSimMotor
        );

        this.rollerMotor = 
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    IntakeConstants.RollerConfigs.kSimMotor,
                    IntakeConstants.RollerConfigs.kSimMotorMOIJ,
                    IntakeConstants.RollerConfigs.kSimMotorGearRatio
                    ),
                IntakeConstants.RollerConfigs.kSimMotor
        );

        this.pivotStatus = new SimMotorStatus(pivotMotor);
        this.rollerStatus = new SimMotorStatus(rollerMotor);
    }

    @Override
    public void zeroHere() {}

    @Override
    public void updatePivotPID(double kA, double kD, double kI, double kP, double kS, double kV) {}

    @Override
    public void setPivotPosition(Angle position) {
        pivotMotor.setAngle(position.in(Radians));
    }

    @Override
    public void setRollerVoltage(Voltage voltage) {
        rollerMotor.setInputVoltage(voltage.in(Volts));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.pivotConnected = pivotStatus.getConnected();
        inputs.pivotPosition = pivotStatus.getPosition();
        inputs.pivotVelocity = pivotStatus.getVelocity();
        inputs.pivotAppliedVoltage = pivotStatus.getAppliedVoltage();
        inputs.pivotStatorCurrent = pivotStatus.getSupplyCurrent();
        inputs.pivotTemperature = pivotStatus.getTemperature();

        inputs.rollerConnected = rollerStatus.getConnected();
        inputs.rollerVelocity = rollerStatus.getVelocity();
        inputs.rollerAppliedVoltage = rollerStatus.getAppliedVoltage();
        inputs.rollerSupplyCurrent = rollerStatus.getSupplyCurrent();
        inputs.rollerStatorCurrent = rollerStatus.getStatorCurrent();
        inputs.rollerTemperature = rollerStatus.getTemperature();
    }
}
