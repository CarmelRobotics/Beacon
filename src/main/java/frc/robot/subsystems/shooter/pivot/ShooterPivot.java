package frc.robot.subsystems.shooter.pivot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.PivotState;
import frc.robot.util.LoggedTunableNumber;

public class ShooterPivot extends SubsystemBase {
    private final ShooterPivotIO io;
    private final ShooterPivotIOInputsAutoLogged inputs;

    private final Alert motorDisconnected;

    private final Supplier<Pose2d> poseSupplier;

    private final LoggedTunableNumber kA;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kCruiseVelocity;
    private final LoggedTunableNumber kAcceleration;
    private final LoggedTunableNumber kJerk;

    public ShooterPivot(ShooterPivotIO io, Supplier<Pose2d> poseSupplier) {
        this.io = io;
        this.inputs = new ShooterPivotIOInputsAutoLogged();

        this.motorDisconnected = new Alert("[Shooter/Pivot] Pivot Motor Disconnected.", AlertType.kError);

        this.poseSupplier = poseSupplier;

        this.kA =
            new LoggedTunableNumber("Shooter/Pivot/kA", ShooterConstants.PivotConfigs.kA);
        this.kD =
            new LoggedTunableNumber("Shooter/Pivot/kD", ShooterConstants.PivotConfigs.kD);
        this.kI =
            new LoggedTunableNumber("Shooter/Pivot/kI", ShooterConstants.PivotConfigs.kI);
        this.kP =
            new LoggedTunableNumber("Shooter/Pivot/kP", ShooterConstants.PivotConfigs.kP);
        this.kS =
            new LoggedTunableNumber("Shooter/Pivot/kS", ShooterConstants.PivotConfigs.kS);
        this.kV =
            new LoggedTunableNumber("Shooter/Pivot/kV", ShooterConstants.PivotConfigs.kV);
        this.kCruiseVelocity = 
            new LoggedTunableNumber("Shooter/Pivot/kCruiseVelocity", ShooterConstants.PivotConfigs.kMotionMagicCruiseVelocity);
        this.kAcceleration = 
            new LoggedTunableNumber("Shooter/Pivot/kAcceleration", ShooterConstants.PivotConfigs.kMotionMagicAcceleration);
        this.kJerk = 
            new LoggedTunableNumber("Shooter/Pivot/kJerk", ShooterConstants.PivotConfigs.kMotionMagicJerk);

        io.zeroHere();
    }

    public Command zeroHere() {
        return run(() -> io.zeroHere());
    }

    public Command setState(PivotState state) {
        return (switch (state) {
            case STOW -> run(() -> io.setPosition(ShooterConstants.kPivotStowAngle));
            case STATIC_SCORE -> run(() -> io.setPosition(ShooterConstants.kPivotStaticScoreAngle));
            /* unimplemented */
            case PASS -> Commands.none();
            case SCORE -> Commands.none();
        }).withName("[Shooter/Pivot] Run from state: " + state.toString());
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
                kV.get()
            );
        }
        if (io instanceof ShooterPivotIOTalonFX && hasMotionMagicChanged()) {
            io.updateMotionMagic(
                kCruiseVelocity.get(),
                kAcceleration.get(),
                kJerk.get()
            );
        }

        motorDisconnected.set(!inputs.connected);
    }
}
