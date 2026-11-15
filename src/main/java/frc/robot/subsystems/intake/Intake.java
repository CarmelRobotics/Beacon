package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeConstants.IntakePivotState;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRollerState;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.pivot.IntakePivotIO;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.roller.IntakeRollerIO;

public class Intake {
  private final IntakePivot pivot;
  private final IntakeRoller roller;

  public Intake(IntakePivotIO pivotio, IntakeRollerIO rollerio, BooleanSupplier readyToFeed) {
    this.pivot = new IntakePivot(pivotio, readyToFeed);
    this.roller = new IntakeRoller(rollerio, pivot::isDeployed);
  }

  public Command setState(Supplier<IntakePivotState> pivotState) {
    return pivot.setState(pivotState);
  }

  public Command setState(IntakePivotState pivotState) {
    return pivot.setState(pivotState);
  }

  public Command setState(IntakeRollerState rollerState) {
    return roller.setState(rollerState);
  }

  public Command setState(IntakePivotState pivotState, IntakeRollerState rollerState) {
    return pivot.setState(pivotState).alongWith(roller.setState(rollerState));
  }

  public void setIdle() {
    roller.setIdle();
  }

  public IntakePivotState getPivotState() {
    return pivot.getState();
  }

  public IntakeRollerState getRollerState() {
    return roller.getState();
  }

  public Command zeroPivotHere() {
    return pivot.zeroHere();
  }
}
