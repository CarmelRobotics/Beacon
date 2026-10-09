package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.motorstatus.TalonFXStatus;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;

  private final TalonFXStatus leaderStatus;
  private final TalonFXStatus followerStatus;

  private final VoltageOut voltageRequest;

  public IntakeRollerIOTalonFX() {
    this.leaderMotor = new TalonFX(IntakeConstants.kLeftLeaderRollerMotorId, Constants.kRIOCANBus);
    this.followerMotor = new TalonFX(IntakeConstants.kRightFollowerRollerMotorId, Constants.kRIOCANBus);

    this.leaderStatus = new TalonFXStatus(leaderMotor);
    this.followerStatus = new TalonFXStatus(followerMotor);

    this.voltageRequest = new VoltageOut(0);

    leaderMotor.getConfigurator().apply(IntakeConstants.RollerConfigs.TalonFXConfiguration);
    followerMotor.getConfigurator().apply(IntakeConstants.RollerConfigs.TalonFXConfiguration);
    followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    leaderStatus.setUpdateFrequency(IntakeConstants.kStatusFrequency);
    followerStatus.setUpdateFrequency(IntakeConstants.kStatusFrequency);
    ParentDevice.optimizeBusUtilizationForAll(leaderMotor, followerMotor);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    leaderMotor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    leaderStatus.refresh();
    followerStatus.refresh();

    inputs.leaderConnected = leaderStatus.getConnected();
    inputs.leaderVelocity = leaderStatus.getVelocity();
    inputs.leaderAppliedVoltage = leaderStatus.getAppliedVoltage();
    inputs.leaderSupplyCurrent = leaderStatus.getSupplyCurrent();
    inputs.leaderStatorCurrent = leaderStatus.getStatorCurrent();
    inputs.leaderTemperature = leaderStatus.getTemperature();

    inputs.followerConnected = followerStatus.getConnected();
    inputs.followerVelocity = followerStatus.getVelocity();
    inputs.followerAppliedVoltage = followerStatus.getAppliedVoltage();
    inputs.followerSupplyCurrent = followerStatus.getSupplyCurrent();
    inputs.followerStatorCurrent = followerStatus.getStatorCurrent();
    inputs.followerTemperature = followerStatus.getTemperature();
  }
}
