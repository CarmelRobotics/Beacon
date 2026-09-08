package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
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
  private final ShooterKicker kicker;
  private final ShooterPivot pivot;
  private final ShooterRoller roller;

  public Shooter(ShooterKickerIO kickerio, ShooterPivotIO pivotio, ShooterRollerIO rollerio,
      Supplier<Pose2d> poseSupplier) {
    this.kicker = new ShooterKicker(kickerio, poseSupplier);
    this.pivot = new ShooterPivot(pivotio, poseSupplier);
    this.roller = new ShooterRoller(rollerio, poseSupplier);
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
