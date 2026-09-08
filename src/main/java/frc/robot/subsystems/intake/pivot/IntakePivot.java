package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.PivotState;
import frc.robot.util.LoggedTunableNumber;

public class IntakePivot extends SubsystemBase {
  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs;

  private final Alert motorDisconnected;

  private final LoggedTunableNumber kA;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kCruiseVelocity;
  private final LoggedTunableNumber kAcceleration;
  private final LoggedTunableNumber kJerk;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
    this.inputs = new IntakePivotIOInputsAutoLogged();

    this.motorDisconnected = new Alert("[Intake/Pivot] Pivot Motor Disconnected.", AlertType.kError);

    this.kA = new LoggedTunableNumber("Intake/Pivot/kA", IntakeConstants.PivotConfigs.kA);
    this.kD = new LoggedTunableNumber("Intake/Pivot/kD", IntakeConstants.PivotConfigs.kD);
    this.kI = new LoggedTunableNumber("Intake/Pivot/kI", IntakeConstants.PivotConfigs.kI);
    this.kP = new LoggedTunableNumber("Intake/Pivot/kP", IntakeConstants.PivotConfigs.kP);
    this.kS = new LoggedTunableNumber("Intake/Pivot/kS", IntakeConstants.PivotConfigs.kS);
    this.kV = new LoggedTunableNumber("Intake/Pivot/kV", IntakeConstants.PivotConfigs.kV);
    this.kCruiseVelocity = new LoggedTunableNumber("Intake/Pivot/kCruiseVelocity",
        IntakeConstants.PivotConfigs.kMotionMagicCruiseVelocity);
    this.kAcceleration = new LoggedTunableNumber("Intake/Pivot/kAcceleration",
        IntakeConstants.PivotConfigs.kMotionMagicAcceleration);
    this.kJerk = new LoggedTunableNumber("Intake/Pivot/kJerk", IntakeConstants.PivotConfigs.kMotionMagicJerk);

    io.zeroHere();
  }

  public Command zeroHere() {
    return run(() -> io.zeroHere());
  }

  public Command setState(PivotState state) {
    return (switch (state) {
      case STOW -> run(() -> io.setAngle(IntakeConstants.kStowAngle));
      case DEPLOY -> run(() -> io.setAngle(IntakeConstants.kDeployAngle));
    }).withName("[Intake/Pivot] Run from state: " + state.toString());
  }

  private boolean hasPIDChanged() {
    return kA.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kI.hasChanged(hashCode())
        || kP.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode());
  }

  private boolean hasMotionMagicChanged() {
    return kCruiseVelocity.hasChanged(hashCode())
        || kAcceleration.hasChanged(hashCode())
        || kJerk.hasChanged(hashCode());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Pivot/Inputs", inputs);

    if (hasPIDChanged()) {
      io.updatePID(
          kA.get(),
          kD.get(),
          kI.get(),
          kP.get(),
          kS.get(),
          kV.get());
    }
    if (hasMotionMagicChanged() && io instanceof IntakePivotIOTalonFX talonFX) {
      talonFX.updateMotionMagic(
          kCruiseVelocity.get(),
          kAcceleration.get(),
          kJerk.get());
    }

    motorDisconnected.set(!inputs.connected);
  }
}
