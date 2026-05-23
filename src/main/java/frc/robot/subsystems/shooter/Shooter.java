package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterState;
import frc.robot.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs;

    /*
    private final LoggedTunableNumber pivotKA;
    private final LoggedTunableNumber pivotKD;
    private final LoggedTunableNumber pivotKI;
    private final LoggedTunableNumber pivotKP;
    private final LoggedTunableNumber pivotKS;
    private final LoggedTunableNumber pivotKV;
    private final LoggedTunableNumber pivotKCruiseVelocity;
    private final LoggedTunableNumber pivotKAcceleration;
    private final LoggedTunableNumber pivotKJerk;

    private final LoggedTunableNumber rollerKA;
    private final LoggedTunableNumber rollerKD;
    private final LoggedTunableNumber rollerKI;
    private final LoggedTunableNumber rollerKP;
    private final LoggedTunableNumber rollerKS;
    private final LoggedTunableNumber rollerKV;

    private final LoggedTunableNumber kickerKA;
    private final LoggedTunableNumber kickerKD;
    private final LoggedTunableNumber kickerKI;
    private final LoggedTunableNumber kickerKP;
    private final LoggedTunableNumber kickerKS;
    private final LoggedTunableNumber kickerKV;
    */

    @AutoLogOutput(key = "Shooter/ShooterState")
    private ShooterState shooterState;

    public Shooter(ShooterIO io, Supplier<Pose2d> poseSupplier) {
        this.io = io;
        this.inputs = new ShooterIOInputsAutoLogged();
        this.shooterState = ShooterState.STOW;
    }
}
