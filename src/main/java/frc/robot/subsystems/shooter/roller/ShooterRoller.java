package frc.robot.subsystems.shooter.roller;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.RollerState;
import frc.robot.util.LoggedTunableNumber;

public class ShooterRoller extends SubsystemBase {
    private final ShooterRollerIO io;
    private final ShooterRollerIOInputsAutoLogged inputs;

    private final Alert frontLeftMotorDisconnected;
    private final Alert backLeftMotorDisconnected;
    private final Alert frontRightMotorDisconnected;
    private final Alert backRightMotorDisconnected;

    private final Supplier<Pose2d> poseSupplier;

    private final LoggedTunableNumber kA;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kV;

    public ShooterRoller(ShooterRollerIO io, Supplier<Pose2d> poseSupplier) {
        this.io = io;
        this.inputs = new ShooterRollerIOInputsAutoLogged();

        this.frontLeftMotorDisconnected = new Alert("[Shooter/Roller] Front Left Roller Motor Disconnected.", AlertType.kError);
        this.backLeftMotorDisconnected = new Alert("[Shooter/Roller] Back Left Roller Motor Disconnected.", AlertType.kError);
        this.frontRightMotorDisconnected = new Alert("[Shooter/Roller] Front Right Roller Motor Disconnected.", AlertType.kError);
        this.backRightMotorDisconnected = new Alert("[Shooter/Roller] Back Right Roller Motor Disconnected.", AlertType.kError);

        this.poseSupplier = poseSupplier;

        this.kA =
            new LoggedTunableNumber("Shooter/Roller/kA", ShooterConstants.PivotConfigs.kA);
        this.kD =
            new LoggedTunableNumber("Shooter/Roller/kD", ShooterConstants.PivotConfigs.kD);
        this.kI =
            new LoggedTunableNumber("Shooter/Roller/kI", ShooterConstants.PivotConfigs.kI);
        this.kP =
            new LoggedTunableNumber("Shooter/Roller/kP", ShooterConstants.PivotConfigs.kP);
        this.kS =
            new LoggedTunableNumber("Shooter/Roller/kS", ShooterConstants.PivotConfigs.kS);
        this.kV =
            new LoggedTunableNumber("Shooter/Roller/kV", ShooterConstants.PivotConfigs.kV);
    }

    public Command setState(RollerState state) {
        return (switch (state) {
            case IDLE -> run(() -> io.setVelocity(ShooterConstants.kRollerIdleVelocity));
            case STATIC_SCORE -> run(() -> io.setVelocity(ShooterConstants.kRollerStaticScoreVelocity));
            case STATIC_SEND -> run(() -> io.setOutput(ShooterConstants.kRollerSendOutput));
            /* unimplemented */
            case PASS -> Commands.none();
            case SCORE -> Commands.none();
        }).withName("[Shooter/Roller] Run from state: " + state.toString());
    }

    private boolean hasPIDChanged() {
        return kA.hasChanged(hashCode())
            || kD.hasChanged(hashCode())
            || kI.hasChanged(hashCode())
            || kP.hasChanged(hashCode())
            || kS.hasChanged(hashCode())
            || kV.hasChanged(hashCode());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/Roller/Inputs", inputs);

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

        frontLeftMotorDisconnected.set(!inputs.frontLeftConnected);
        backLeftMotorDisconnected.set(!inputs.backLeftConnected);
        frontRightMotorDisconnected.set(!inputs.frontRightConnected);
        backRightMotorDisconnected.set(!inputs.backRightConnected);
    }
}
