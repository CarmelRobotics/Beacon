package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.FieldRelative;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterKickerState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterPivotState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterRollerState;

public class Auto {
  private final Drive drive;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;
  private final FieldRelative field;
  private final SendableChooser<Command> chooser;

  public Auto(Drive drive, Shooter shooter, Indexer indexer, Intake intake, FieldRelative field) {
    this.drive = drive;
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
    this.field = field;
    this.chooser = new SendableChooser<>();

    chooser.addOption("EightAuto", eightAuto());
  }

  public SendableChooser<Command> getChooser() {
    return this.chooser;
  } 

  public Command eightAuto() {
    final double rotateCoeff =
      Constants.kCurrentMode == Mode.kReal
      ? 0.3
      : 0.2;

    return Commands.sequence(
      drive.seedFieldCentric(),

      drive.run(() -> drive.runVelocity(new ChassisSpeeds(-1.0, 0.0, 0.0)))
        .withTimeout(Seconds.of(1)),
      
      drive.runOnce(drive::stop),
      Commands.waitSeconds(0.15),

      drive.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, 0.0, rotateCoeff * drive.getMaxAngularSpeedRadPerSec())))
        .withTimeout(Seconds.of(1)),

      drive.runOnce(drive::stop),
      Commands.waitSeconds(0.15),

      shooter.setState(ShooterKickerState.kFeed, ShooterPivotState.kStaticScore, ShooterRollerState.kStaticScore)
        .alongWith(indexer.setState(IndexerState.kFeed)),
        
      Commands.waitSeconds(15.0),

      indexer.setState(IndexerState.kIdle)
        .alongWith(shooter.setState(ShooterKickerState.kIdle, ShooterPivotState.kStow, ShooterRollerState.kIdle))
    );
  }
}
