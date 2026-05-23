package frc.robot.subsystems.shooter.pivot;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.SimMotorStatus;

public class ShooterPivotIOSim implements ShooterPivotIO {
    private final DCMotorSim pivotMotor;
    private final SimMotorStatus pivotStatus;

    public ShooterPivotIOSim() {
        this.pivotMotor = 
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    ShooterConstants.PivotConfigs.kSimMotor,
                    ShooterConstants.PivotConfigs.kSimMotorMOIJ,
                    ShooterConstants.PivotConfigs.kSimMotorGearRatio
                    ),
                ShooterConstants.PivotConfigs.kSimMotor
        );

        this.pivotStatus = new SimMotorStatus(pivotMotor);
    }

    @Override
    public void zeroPivotHere() {}

    @Override
    public void updatePivotPID(double kA, double kD, double kI, double kP, double kS, double kV) {}
    
    @Override
    public void setPivotPosition(Angle position) {
        pivotMotor.setAngle(position.in(Radians));
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
