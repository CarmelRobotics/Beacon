package frc.robot.subsystems.shooter.pivot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterPivotState;
import frc.robot.util.LoggedTunableNumber;

public class ShooterPivot extends SubsystemBase {
  private final ShooterPivotIO io;
  private final ShooterPivotIOInputsAutoLogged inputs;

  private final Alert motorDisconnected;

  private final DoubleSupplier targetDistance;

  private final LoggedTunableNumber kA;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kCruiseVelocity;
  private final LoggedTunableNumber kAcceleration;
  private final LoggedTunableNumber kJerk;

  @AutoLogOutput(key = "Shooter/Pivot/Position")
  private Angle targetPosition;
  @AutoLogOutput(key = "Shooter/Pivot/State")
  private ShooterPivotState state;

  public ShooterPivot(ShooterPivotIO io, DoubleSupplier targetDistanceSupplier) {
    this.io = io;
    this.inputs = new ShooterPivotIOInputsAutoLogged();

    this.motorDisconnected = new Alert("[Shooter/Pivot] Pivot Motor Disconnected.", AlertType.kError);

    this.targetDistance = targetDistanceSupplier;

    this.kA = new LoggedTunableNumber("Shooter/Pivot/kA", ShooterConstants.PivotConfigs.kA);
    this.kD = new LoggedTunableNumber("Shooter/Pivot/kD", ShooterConstants.PivotConfigs.kD);
    this.kI = new LoggedTunableNumber("Shooter/Pivot/kI", ShooterConstants.PivotConfigs.kI);
    this.kP = new LoggedTunableNumber("Shooter/Pivot/kP", ShooterConstants.PivotConfigs.kP);
    this.kS = new LoggedTunableNumber("Shooter/Pivot/kS", ShooterConstants.PivotConfigs.kS);
    this.kV = new LoggedTunableNumber("Shooter/Pivot/kV", ShooterConstants.PivotConfigs.kV);
    this.kCruiseVelocity = new LoggedTunableNumber("Shooter/Pivot/kCruiseVelocity",
        ShooterConstants.PivotConfigs.kMotionMagicCruiseVelocity);
    this.kAcceleration = new LoggedTunableNumber("Shooter/Pivot/kAcceleration",
        ShooterConstants.PivotConfigs.kMotionMagicAcceleration);
    this.kJerk = new LoggedTunableNumber("Shooter/Pivot/kJerk", ShooterConstants.PivotConfigs.kMotionMagicJerk);

    this.targetPosition = Degrees.zero();
    this.state = ShooterPivotState.kStow;

    io.zeroHere();
  }

  public Command zeroHere() {
    return run(() -> io.zeroHere());
  }

  public Command setState(ShooterPivotState state) {
    return (switch (state) {
      case kStow -> runOnce(() -> setPosition(ShooterConstants.kPivotStowAngle));
      case kStaticScore -> runOnce(() -> setPosition(ShooterConstants.kPivotStaticScoreAngle));
      case kPass -> run(() -> setPosition(ShooterConstants.kPivotPassAngle));
      case kScore -> run(() -> setPosition(Degrees.of(ShooterConstants.PivotScoringMap.get(targetDistance.getAsDouble()))));
    }).beforeStarting(() -> {this.state = state;})
      .withName("[Shooter/Pivot] Run from state: " + state.toString());
  }

  public ShooterPivotState getState() {
    return state;
  }

  private void setPosition(Angle position) {
    targetPosition = position;
    io.setPosition(position);
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
    Logger.processInputs("Shooter/Pivot/Inputs", inputs);

    if (hasPIDChanged()) {
      io.updatePID(
          kA.get(),
          kD.get(),
          kI.get(),
          kP.get(),
          kS.get(),
          kV.get());
    }
    if (io instanceof ShooterPivotIOTalonFX && hasMotionMagicChanged()) {
      ShooterPivotIOTalonFX talonFXIO = (ShooterPivotIOTalonFX) io;
      talonFXIO.updateMotionMagic(
          kCruiseVelocity.get(),
          kAcceleration.get(),
          kJerk.get());
    }

    motorDisconnected.set(!inputs.connected);
  }
}
