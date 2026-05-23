package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;
import frc.robot.subsystems.shooter.kicker.ShooterKickerIO;
import frc.robot.subsystems.shooter.kicker.ShooterKickerIOInputsAutoLogged;
import frc.robot.subsystems.shooter.pivot.ShooterPivotIO;
import frc.robot.subsystems.shooter.pivot.ShooterPivotIOInputsAutoLogged;
import frc.robot.subsystems.shooter.pivot.ShooterPivotIOTalonFX;
import frc.robot.subsystems.shooter.roller.ShooterRollerIO;
import frc.robot.subsystems.shooter.roller.ShooterRollerIOInputsAutoLogged;
import frc.robot.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
    private final ShooterKickerIO kickerIO;
    private final ShooterPivotIO pivotIO;
    private final ShooterRollerIO rollerIO;

    private final ShooterKickerIOInputsAutoLogged kickerInputs;
    private final ShooterPivotIOInputsAutoLogged pivotInputs;
    private final ShooterRollerIOInputsAutoLogged rollerInputs;

    private final Supplier<Pose2d> poseSupplier;

    @AutoLogOutput(key = "Shooter/ShooterState")
    private ShooterState shooterState;
    
    private final LoggedTunableNumber kickerKA;
    private final LoggedTunableNumber kickerKD;
    private final LoggedTunableNumber kickerKI;
    private final LoggedTunableNumber kickerKP;
    private final LoggedTunableNumber kickerKS;
    private final LoggedTunableNumber kickerKV;    

    private final LoggedTunableNumber rollerKA;
    private final LoggedTunableNumber rollerKD;
    private final LoggedTunableNumber rollerKI;
    private final LoggedTunableNumber rollerKP;
    private final LoggedTunableNumber rollerKS;
    private final LoggedTunableNumber rollerKV;

    private final LoggedTunableNumber pivotKA;
    private final LoggedTunableNumber pivotKD;
    private final LoggedTunableNumber pivotKI;
    private final LoggedTunableNumber pivotKP;
    private final LoggedTunableNumber pivotKS;
    private final LoggedTunableNumber pivotKV;
    private final LoggedTunableNumber pivotKCruiseVelocity;
    private final LoggedTunableNumber pivotKAcceleration;
    private final LoggedTunableNumber pivotKJerk;

    private final Alert kickerMotorDisconnected;
    private final Alert pivotMotorDisconnected;
    private final Alert frontLeftMotorDisconnected;
    private final Alert backLeftMotorDisconnected;
    private final Alert frontRightMotorDisconnected;
    private final Alert backRightMotorDisconnected;
    
    public Shooter(ShooterKickerIO kickerIO, ShooterPivotIO pivotIO, ShooterRollerIO rollerIO, Supplier<Pose2d> poseSupplier) {
        this.kickerIO = kickerIO;
        this.pivotIO = pivotIO;
        this.rollerIO = rollerIO;

        this.kickerInputs = new ShooterKickerIOInputsAutoLogged();
        this.pivotInputs = new ShooterPivotIOInputsAutoLogged();
        this.rollerInputs = new ShooterRollerIOInputsAutoLogged();

        this.poseSupplier = poseSupplier;

        this.shooterState = ShooterState.STOW;

        this.kickerKA =
            new LoggedTunableNumber("Shooter/kickerKA", ShooterConstants.PivotConfigs.kA);
        this.kickerKD =
            new LoggedTunableNumber("Shooter/kickerKD", ShooterConstants.PivotConfigs.kD);
        this.kickerKI =
            new LoggedTunableNumber("Shooter/kickerKI", ShooterConstants.PivotConfigs.kI);
        this.kickerKP =
            new LoggedTunableNumber("Shooter/kickerKP", ShooterConstants.PivotConfigs.kP);
        this.kickerKS =
            new LoggedTunableNumber("Shooter/kickerKS", ShooterConstants.PivotConfigs.kS);
        this.kickerKV =
            new LoggedTunableNumber("Shooter/kickerKV", ShooterConstants.PivotConfigs.kV);

        this.rollerKA =
            new LoggedTunableNumber("Shooter/rollerKA", ShooterConstants.PivotConfigs.kA);
        this.rollerKD =
            new LoggedTunableNumber("Shooter/rollerKD", ShooterConstants.PivotConfigs.kD);
        this.rollerKI =
            new LoggedTunableNumber("Shooter/rollerKI", ShooterConstants.PivotConfigs.kI);
        this.rollerKP =
            new LoggedTunableNumber("Shooter/rollerKP", ShooterConstants.PivotConfigs.kP);
        this.rollerKS =
            new LoggedTunableNumber("Shooter/rollerKS", ShooterConstants.PivotConfigs.kS);
        this.rollerKV =
            new LoggedTunableNumber("Shooter/rollerKV", ShooterConstants.PivotConfigs.kV);

        this.pivotKA =
            new LoggedTunableNumber("Shooter/pivotKA", ShooterConstants.PivotConfigs.kA);
        this.pivotKD =
            new LoggedTunableNumber("Shooter/pivotKD", ShooterConstants.PivotConfigs.kD);
        this.pivotKI =
            new LoggedTunableNumber("Shooter/pivotKI", ShooterConstants.PivotConfigs.kI);
        this.pivotKP =
            new LoggedTunableNumber("Shooter/pivotKP", ShooterConstants.PivotConfigs.kP);
        this.pivotKS =
            new LoggedTunableNumber("Shooter/pivotKS", ShooterConstants.PivotConfigs.kS);
        this.pivotKV =
            new LoggedTunableNumber("Shooter/pivotKV", ShooterConstants.PivotConfigs.kV);
        this.pivotKCruiseVelocity = 
            new LoggedTunableNumber("Shooter/pivotKCruiseVelocity", ShooterConstants.PivotConfigs.kMotionMagicCruiseVelocity);
        this.pivotKAcceleration = 
            new LoggedTunableNumber("Shooter/pivotKAcceleration", ShooterConstants.PivotConfigs.kMotionMagicAcceleration);
        this.pivotKJerk = 
            new LoggedTunableNumber("Shooter/pivotKJerk", ShooterConstants.PivotConfigs.kMotionMagicJerk);

        this.kickerMotorDisconnected = new Alert("[Shooter] Kicker Motor Disconnected.", AlertType.kError);
        this.pivotMotorDisconnected = new Alert("[Shooter] Pivot Motor Disconnected.", AlertType.kError);
        this.frontLeftMotorDisconnected = new Alert("[Shooter] Front Left Roller Motor Disconnected.", AlertType.kError);
        this.backLeftMotorDisconnected = new Alert("[Shooter] Back Left Roller Motor Disconnected.", AlertType.kError);
        this.frontRightMotorDisconnected = new Alert("[Shooter] Front Right Roller Motor Disconnected.", AlertType.kError);
        this.backRightMotorDisconnected = new Alert("[Shooter] Back Right Roller Motor Disconnected.", AlertType.kError);
    }

    private boolean hasKickerPIDChanged() {
        return kickerKA.hasChanged(hashCode())
            || kickerKD.hasChanged(hashCode())
            || kickerKI.hasChanged(hashCode())
            || kickerKP.hasChanged(hashCode())
            || kickerKS.hasChanged(hashCode())
            || kickerKV.hasChanged(hashCode());
    }

    private boolean hasRollerPIDChanged() {
        return rollerKA.hasChanged(hashCode())
            || rollerKD.hasChanged(hashCode())
            || rollerKI.hasChanged(hashCode())
            || rollerKP.hasChanged(hashCode())
            || rollerKS.hasChanged(hashCode())
            || rollerKV.hasChanged(hashCode());
    }

    private boolean hasPivotPIDChanged() {
        return pivotKA.hasChanged(hashCode())
            || pivotKD.hasChanged(hashCode())
            || pivotKI.hasChanged(hashCode())
            || pivotKP.hasChanged(hashCode())
            || pivotKS.hasChanged(hashCode())
            || pivotKV.hasChanged(hashCode());
    }

    private boolean hasPivotMotionMagicChanged() {
        return pivotKCruiseVelocity.hasChanged(hashCode())
            || pivotKAcceleration.hasChanged(hashCode())
            || pivotKJerk.hasChanged(hashCode());
    }

    public void zeroPivotHere() {
        pivotIO.zeroPivotHere();
    }

    public void setState(ShooterState shooterState) {
        this.shooterState = shooterState;
    }

    @Override
    public void periodic() {
        kickerIO.updateInputs(kickerInputs);
        pivotIO.updateInputs(pivotInputs);
        rollerIO.updateInputs(rollerInputs);

        Logger.processInputs("Shooter/KickerInputs", kickerInputs);
        Logger.processInputs("Shooter/PivotInputs", pivotInputs);
        Logger.processInputs("Shooter/RollerInputs", rollerInputs);

        if (hasKickerPIDChanged()) {
            kickerIO.updatePID(
                kickerKA.get(),
                kickerKD.get(),
                kickerKI.get(),
                kickerKP.get(),
                kickerKS.get(),
                kickerKV.get()
            );
        }
        if (hasRollerPIDChanged()) {
            rollerIO.updatePID(
                rollerKA.get(),
                rollerKD.get(),
                rollerKI.get(),
                rollerKP.get(),
                rollerKS.get(),
                rollerKV.get()
            );
        }
        if (hasPivotPIDChanged()) {
            pivotIO.updatePivotPID(
                pivotKA.get(),
                pivotKD.get(),
                pivotKI.get(),
                pivotKP.get(),
                pivotKS.get(),
                pivotKV.get()
            );
        }
        if (pivotIO instanceof ShooterPivotIOTalonFX && hasPivotMotionMagicChanged()) {
            pivotIO.updatePivotMotionMagic(
                pivotKCruiseVelocity.get(),
                pivotKAcceleration.get(),
                pivotKJerk.get()
            );
        }

        switch (shooterState) {
            case STOW -> {
                kickerIO.setVoltage(Volts.zero());
                rollerIO.setVoltage(Volts.zero());
                pivotIO.setPivotPosition(Degrees.zero());
            }
            case OFF -> {
                kickerIO.setVoltage(Volts.zero());
                rollerIO.setVoltage(Volts.zero());
            }
            case PASS_PRESET -> {
                
            }
            case PASS -> {

            }
            case SCORE_PRESET -> {

            }
            case SCORE -> {

            }
        }

        kickerMotorDisconnected.set(!kickerInputs.connected);
        pivotMotorDisconnected.set(!pivotInputs.connected);
        frontLeftMotorDisconnected.set(!rollerInputs.frontLeftConnected);
        backLeftMotorDisconnected.set(!rollerInputs.backLeftConnected);
        frontRightMotorDisconnected.set(!rollerInputs.frontRightConnected);
        backRightMotorDisconnected.set(!rollerInputs.backRightConnected);
    }
}
