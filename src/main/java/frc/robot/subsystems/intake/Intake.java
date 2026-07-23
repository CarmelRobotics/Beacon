package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeConstants.PivotState;
import frc.robot.subsystems.intake.IntakeConstants.RollerState;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.pivot.IntakePivotIO;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.roller.IntakeRollerIO;

public class Intake {
    private final IntakePivot pivot;
    private final IntakeRoller roller;

    public Intake(IntakePivotIO pivotio, IntakeRollerIO rollerio) {
        this.pivot = new IntakePivot(pivotio);
        this.roller = new IntakeRoller(rollerio);
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
