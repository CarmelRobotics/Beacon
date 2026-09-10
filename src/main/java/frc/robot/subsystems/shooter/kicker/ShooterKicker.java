package frc.robot.subsystems.shooter.kicker;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterKickerState;
import frc.robot.util.LoggedTunableNumber;

public class ShooterKicker extends SubsystemBase {
  private final ShooterKickerIO io;
  private final ShooterKickerIOInputsAutoLogged inputs;

  private final Alert motorDisconnected;

  private final BooleanSupplier readyToFeed;
  private final Supplier<AngularVelocity> rollerVelocity;

  private final LoggedTunableNumber kA;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;

  @AutoLogOutput(key = "Shooter/Kicker/Velocity")
  private AngularVelocity targetVelocity;
  @AutoLogOutput(key = "Shooter/Kicker/Output")
  private double targetOutput;
  @AutoLogOutput(key = "Shooter/Kicker/State")
  private ShooterKickerState state;

  public ShooterKicker(
    ShooterKickerIO io,
    BooleanSupplier readyToFeed,
    Supplier<AngularVelocity> rollerVelocity) {
    this.io = io;
    this.inputs = new ShooterKickerIOInputsAutoLogged();

    this.motorDisconnected = new Alert("[Shooter/Kicker] Kicker Motor Disconnected.", AlertType.kError);

    this.readyToFeed = readyToFeed;
    this.rollerVelocity = rollerVelocity;

    this.kA = new LoggedTunableNumber("Shooter/Kicker/kA", ShooterConstants.PivotConfigs.kA);
    this.kD = new LoggedTunableNumber("Shooter/Kicker/kD", ShooterConstants.PivotConfigs.kD);
    this.kI = new LoggedTunableNumber("Shooter/Kicker/kI", ShooterConstants.PivotConfigs.kI);
    this.kP = new LoggedTunableNumber("Shooter/Kicker/kP", ShooterConstants.PivotConfigs.kP);
    this.kS = new LoggedTunableNumber("Shooter/Kicker/kS", ShooterConstants.PivotConfigs.kS);
    this.kV = new LoggedTunableNumber("Shooter/Kicker/kV", ShooterConstants.PivotConfigs.kV);

    this.targetVelocity = RotationsPerSecond.zero();
    this.targetOutput = 0.0;
    this.state = ShooterKickerState.kIdle;
  }

  public Command setState(ShooterKickerState state) {
    return (switch (state) {
      case kIdle -> runOnce(() -> setOutput(ShooterConstants.kKickerIdleOutput));
      case kEject -> runOnce(() -> setOutput(ShooterConstants.kKickerEjectOutput));
      case kStaticScore -> runOnce(() -> setVelocity(ShooterConstants.kKickerStaticScoreVelocity));
      case kPass, kScore -> 
        Commands.waitUntil(readyToFeed)
          .andThen(run(() -> setVelocity(RotationsPerSecond.of(
            ShooterConstants.KickerMap.get(
              rollerVelocity.get().in(RotationsPerSecond))))));
    }).beforeStarting(() -> {this.state = state;})
      .withName("[Shooter/Kicker] Run from state: " + state.toString());
  }

  public ShooterKickerState getState() {
    return state;
  }

  private void setVelocity(AngularVelocity velocity) {
    targetOutput = 0.0;
    targetVelocity = velocity;
    io.setVelocity(velocity);
  }

  private void setOutput(double output) {
    targetVelocity = RotationsPerSecond.zero();
    targetOutput = output;
    io.setOutput(output);
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
