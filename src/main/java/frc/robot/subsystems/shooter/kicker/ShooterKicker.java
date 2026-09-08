package frc.robot.subsystems.shooter.kicker;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.KickerState;
import frc.robot.util.LoggedTunableNumber;

public class ShooterKicker extends SubsystemBase {
  private final ShooterKickerIO io;
  private final ShooterKickerIOInputsAutoLogged inputs;

  private final Alert motorDisconnected;

  private final Supplier<Pose2d> poseSupplier;

  private final LoggedTunableNumber kA;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;

  public ShooterKicker(ShooterKickerIO io, Supplier<Pose2d> poseSupplier) {
    this.io = io;
    this.inputs = new ShooterKickerIOInputsAutoLogged();

    this.motorDisconnected = new Alert("[Shooter/Kicker] Kicker Motor Disconnected.", AlertType.kError);

    this.poseSupplier = poseSupplier;

    this.kA = new LoggedTunableNumber("Shooter/Kicker/kA", ShooterConstants.PivotConfigs.kA);
    this.kD = new LoggedTunableNumber("Shooter/Kicker/kD", ShooterConstants.PivotConfigs.kD);
    this.kI = new LoggedTunableNumber("Shooter/Kicker/kI", ShooterConstants.PivotConfigs.kI);
    this.kP = new LoggedTunableNumber("Shooter/Kicker/kP", ShooterConstants.PivotConfigs.kP);
    this.kS = new LoggedTunableNumber("Shooter/Kicker/kS", ShooterConstants.PivotConfigs.kS);
    this.kV = new LoggedTunableNumber("Shooter/Kicker/kV", ShooterConstants.PivotConfigs.kV);
  }

  public Command setState(KickerState state) {
    return (switch (state) {
      case IDLE -> run(() -> io.setOutput(ShooterConstants.kKickerIdleOutput));
      case EJECT -> run(() -> io.setOutput(ShooterConstants.kKickerEjectOutput));
      case STATIC_SCORE -> run(() -> io.setVelocity(ShooterConstants.kKickerStaticScoreVelocity));
      /* unimplemented */
      case PASS -> Commands.none();
      case SCORE -> Commands.none();
    }).withName("[Shooter/Kicker] Run from state: " + state.toString());
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
    Logger.processInputs("Shooter/Kicker/Inputs", inputs);

    if (hasPIDChanged()) {
      io.updatePID(
          kA.get(),
          kD.get(),
          kI.get(),
          kP.get(),
          kS.get(),
          kV.get());
    }

    motorDisconnected.set(!inputs.connected);
  }
}
