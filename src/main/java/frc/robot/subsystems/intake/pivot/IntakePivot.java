package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.IntakePivotState;
import frc.robot.util.LoggedTunableNumber;

public class IntakePivot extends SubsystemBase {
  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs;

  private final Alert leaderDisconnected;
  private final Alert followerDisconnected;

  private final LoggedTunableNumber kA;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kCruiseVelocity;
  private final LoggedTunableNumber kAcceleration;
  private final LoggedTunableNumber kJerk;

  @AutoLogOutput(key = "Intake/Pivot/Position")
  private Angle targetPosition;
  @AutoLogOutput(key = "Intake/Pivot/State")
  private IntakePivotState state;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
    this.inputs = new IntakePivotIOInputsAutoLogged();

    this.leaderDisconnected = new Alert("[Intake/Pivot] Leader Pivot Motor Disconnected.", AlertType.kError);
    this.followerDisconnected = new Alert("[Intake/Pivot] Follower Pivot Motor Disconnected.", AlertType.kError);

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

    this.targetPosition = Degrees.zero();
    this.state = IntakePivotState.kStow;
  }

  public Command zeroHere() {
    return run(() -> io.zeroHere());
  }

  public Command setState(IntakePivotState state) {
    return (switch (state) {
      case kStow -> runOnce(() -> setPosition(IntakeConstants.kStowAngle));
      case kDeploy -> runOnce(() -> setPosition(IntakeConstants.kDeployAngle));
    }).beforeStarting(() -> {this.state = state;})
      .withName("[Intake/Pivot] Run from state: " + state.toString());
  }

  public IntakePivotState getState() {
    return state;
  }

  private void setPosition(Angle position) {
    io.setPosition(targetPosition);
    targetPosition = position;
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

    leaderDisconnected.set(!inputs.leaderConnected);
    followerDisconnected.set(!inputs.followerConnected);
  }
}
