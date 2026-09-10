package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.RollerState;

public class IntakeRoller extends SubsystemBase {
  private final IntakeRollerIO io;
  private final IntakeRollerIOInputsAutoLogged inputs;

  private final Alert motorDisconnected;

  @AutoLogOutput
  private Voltage targetVoltage;

  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
    this.inputs = new IntakeRollerIOInputsAutoLogged();

    this.motorDisconnected = new Alert("[Intake/Roller] Roller Motor Disconnected.", AlertType.kError);

    this.targetVoltage = Volts.zero();
  }

  public Command setState(RollerState state) {
    return (switch (state) {
      case kIdle -> run(() -> setVoltage(IntakeConstants.kIdleVoltage));
      case kIntake -> run(() -> setVoltage(IntakeConstants.kIntakeVoltage));
      case kEject -> run(() -> setVoltage(IntakeConstants.kEjectVoltage));
    }).withName("[Intake/Roller] Run from state: " + state.toString());
  }

  private void setVoltage(Voltage voltage) {
    targetVoltage = voltage;
    io.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Roller/Inputs", inputs);
    motorDisconnected.set(!inputs.connected);
  }
}
