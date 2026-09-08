package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.motorstatus.SimMotorStatus;

public class IntakePivotIOSim implements IntakePivotIO {
  private final DCMotorSim pivotMotor;
  private final SimMotorStatus pivotStatus;

  public IntakePivotIOSim() {
    this.pivotMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            IntakeConstants.PivotConfigs.kSimMotor,
            IntakeConstants.PivotConfigs.kSimMotorMOIJ,
            IntakeConstants.PivotConfigs.kSimMotorGearRatio),
        IntakeConstants.PivotConfigs.kSimMotor);
    this.pivotStatus = new SimMotorStatus(pivotMotor);
  }

  @Override
  public void zeroHere() {
  }

  @Override
  public void updatePID(double kA, double kD, double kI, double kP, double kS, double kV) {
  }

  @Override
  public void setAngle(Angle angle) {
    pivotMotor.setAngle(angle.in(Radians));
  }

  @Override
  public void updateInputs(IntakePivotIOInputsAutoLogged inputs) {
    inputs.connected = pivotStatus.getConnected();
    inputs.position = pivotStatus.getPosition();
    inputs.velocity = pivotStatus.getVelocity();
    inputs.appliedVoltage = pivotStatus.getAppliedVoltage();
    inputs.statorCurrent = pivotStatus.getSupplyCurrent();
    inputs.temperature = pivotStatus.getTemperature();
  }
}
