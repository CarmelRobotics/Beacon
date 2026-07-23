package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeConstants.PivotState;
import frc.robot.subsystems.intake.IntakeConstants.RollerState;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.roller.IntakeRoller;

public class Intake {
    private final IntakePivot pivot;
    private final IntakeRoller roller;

    public Intake(IntakePivot pivot, IntakeRoller roller) {
        this.pivot = pivot;
        this.roller = roller;
    }

    public Command setState(PivotState pivotState) {
        return pivot.setState(pivotState);
    }

    public Command setState(RollerState rollerState) {
        return roller.setState(rollerState);
    }

    public Command setState(PivotState pivotState, RollerState rollerState) {
        return pivot.setState(pivotState).alongWith(roller.setState(rollerState)); 
    }

    public Command zeroPivotHere() {
        return pivot.zeroHere();
    }
}
