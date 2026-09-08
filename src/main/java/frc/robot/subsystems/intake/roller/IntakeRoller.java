package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.Logger;

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

  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
    this.inputs = new IntakeRollerIOInputsAutoLogged();

    this.motorDisconnected = new Alert("[Intake/Roller] Roller Motor Disconnected.", AlertType.kError);
  }

  public Command setState(RollerState state) {
    return (switch (state) {
      case IDLE -> run(() -> io.setVoltage(IntakeConstants.kIdleVoltage));
      case INTAKE -> run(() -> io.setVoltage(IntakeConstants.kIntakeVoltage));
      case EJECT -> run(() -> io.setVoltage(IntakeConstants.kEjectVoltage));
    }).withName("[Intake/Roller] Run from state: " + state.toString());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Roller/Inputs", inputs);
    motorDisconnected.set(!inputs.connected);
  }
}
