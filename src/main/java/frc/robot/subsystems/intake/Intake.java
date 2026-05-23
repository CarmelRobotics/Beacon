package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.PivotState;
import frc.robot.subsystems.intake.IntakeIO.RollerState;
import frc.robot.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;

    @AutoLogOutput(key = "Intake/PivotState")
    private PivotState pivotState;

    @AutoLogOutput(key = "Intake/RollerState")
    private RollerState rollerState;

    private final LoggedTunableNumber pivotKA;
    private final LoggedTunableNumber pivotKD;
    private final LoggedTunableNumber pivotKI;
    private final LoggedTunableNumber pivotKP;
    private final LoggedTunableNumber pivotKS;
    private final LoggedTunableNumber pivotKV;
    private final LoggedTunableNumber pivotKCruiseVelocity;
    private final LoggedTunableNumber pivotKAcceleration;
    private final LoggedTunableNumber pivotKJerk;

    private final Alert pivotMotorDisconnected;
    private final Alert rollerMotorDisconnected;

    public Intake(IntakeIO io) {
        this.io = io;
        this.inputs = new IntakeIOInputsAutoLogged();
        this.pivotState = PivotState.STOW;
        this.rollerState = RollerState.OFF;

        this.pivotKA = 
            new LoggedTunableNumber("Intake/pivotKA", IntakeConstants.PivotConfigs.kA);
        this.pivotKD = 
            new LoggedTunableNumber("Intake/pivotKD", IntakeConstants.PivotConfigs.kD);
        this.pivotKI = 
            new LoggedTunableNumber("Intake/pivotKI", IntakeConstants.PivotConfigs.kI);
        this.pivotKP = 
            new LoggedTunableNumber("Intake/pivotKP", IntakeConstants.PivotConfigs.kP);
        this.pivotKS = 
            new LoggedTunableNumber("Intake/pivotKS", IntakeConstants.PivotConfigs.kS);
        this.pivotKV = 
            new LoggedTunableNumber("Intake/pivotKV", IntakeConstants.PivotConfigs.kV);
        this.pivotKCruiseVelocity = 
            new LoggedTunableNumber("Intake/pivotKCruiseVelocity", IntakeConstants.PivotConfigs.kMotionMagicCruiseVelocity);
        this.pivotKAcceleration = 
            new LoggedTunableNumber("Intake/pivotKAcceleration", IntakeConstants.PivotConfigs.kMotionMagicAcceleration);
        this.pivotKJerk = 
            new LoggedTunableNumber("Intake/pivotKJerk", IntakeConstants.PivotConfigs.kMotionMagicJerk);

        this.pivotMotorDisconnected = new Alert("[Intake] Pivot Motor Disconnected.", AlertType.kError);
        this.rollerMotorDisconnected = new Alert("[Intake] Roller Motor Disconnected.", AlertType.kError);
    }

    private boolean hasPIDChanged() {
        return pivotKA.hasChanged(hashCode())
            || pivotKD.hasChanged(hashCode())
            || pivotKI.hasChanged(hashCode())
            || pivotKP.hasChanged(hashCode())
            || pivotKS.hasChanged(hashCode())
            || pivotKV.hasChanged(hashCode());
    }

    private boolean hasMotionMagicChanged() {
        return pivotKCruiseVelocity.hasChanged(hashCode())
            || pivotKAcceleration.hasChanged(hashCode())
            || pivotKJerk.hasChanged(hashCode());
    }

    public void zeroPivotHere() {
        io.zeroPivotHere();
    }

    public void setState(PivotState pivotState, RollerState rollerState) {
        this.pivotState = pivotState;
        this.rollerState = rollerState;
    }

    public void setState(PivotState pivotState) {
        this.pivotState = pivotState;
    }

    public void setState(RollerState rollerState) {
        this.rollerState = rollerState;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Inputs", inputs);

        if (hasPIDChanged()) {
            io.updatePivotPID(
                pivotKA.get(),
                pivotKD.get(),
                pivotKI.get(),
                pivotKP.get(),
                pivotKS.get(),
                pivotKV.get()
            );
        }
        if (io instanceof IntakeIOTalonFX && hasMotionMagicChanged()) {
            io.updatePivotMotionMagic(
                pivotKCruiseVelocity.get(),
                pivotKCruiseVelocity.get(),
                pivotKJerk.get()
            );
        }

        io.setPivotPosition(pivotState.position);
        io.setRollerVoltage(rollerState.voltage);

        pivotMotorDisconnected.set(!inputs.pivotConnected);
        rollerMotorDisconnected.set(!inputs.rollerConnected);
    }
}
