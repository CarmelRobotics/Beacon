package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterConstants.KickerState;
import frc.robot.subsystems.shooter.ShooterConstants.PivotState;
import frc.robot.subsystems.shooter.ShooterConstants.RollerState;
import frc.robot.subsystems.shooter.kicker.ShooterKicker;
import frc.robot.subsystems.shooter.kicker.ShooterKickerIO;
import frc.robot.subsystems.shooter.pivot.ShooterPivot;
import frc.robot.subsystems.shooter.pivot.ShooterPivotIO;
import frc.robot.subsystems.shooter.roller.ShooterRoller;
import frc.robot.subsystems.shooter.roller.ShooterRollerIO;

public class Shooter {
  private final ShooterPivot pivot;
  private final ShooterRoller roller;
  private final ShooterKicker kicker;

  public Shooter(
    ShooterKickerIO kickerio,
    ShooterPivotIO pivotio,
    ShooterRollerIO rollerio,
    BooleanSupplier facingTarget,
    DoubleSupplier targetDistance) {
    this.pivot = new ShooterPivot(pivotio, targetDistance);
    this.roller = new ShooterRoller(rollerio, facingTarget, targetDistance);
    this.kicker = new ShooterKicker(kickerio, roller::isReadyToScore, roller::getTargetVelocity);
  }

  public Command setState(KickerState state) {
    return kicker.setState(state);
  }

  public Command setState(PivotState state) {
    return pivot.setState(state);
  }

  public Command setState(RollerState state) {
    return roller.setState(state);
  }

  public Command setState(KickerState kickerState, PivotState pivotState, RollerState rollerState) {
    return kicker.setState(kickerState).alongWith(pivot.setState(pivotState)).alongWith(roller.setState(rollerState));
  }

  public Command zeroPivotHere() {
    return pivot.zeroHere();
  }
}
