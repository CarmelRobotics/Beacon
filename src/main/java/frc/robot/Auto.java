package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    chooser.addOption("LarpAuto", larpAuto());
    chooser.addOption("None", Commands.none());

    SmartDashboard.putBoolean("Auto/MirrorPath", false);
    SmartDashboard.putData("Auto/AutoChooser", chooser);
  }

  public Command getSelected() {
    return chooser.getSelected();
  }

  public Command larpAuto() {
    return Commands.sequence(
        drive.seedFieldCentric(),

        drive.run(() -> drive.runVelocity(new ChassisSpeeds(-1.0, 0.0, 0.0)))
            .withTimeout(Seconds.of(1)),

        drive.runOnce(drive::stop),
        drive.targetedDrive(
            () -> 0,
            () -> 0,
            0.0,
            field::hubTarget).until(drive::isFacingTarget),
        drive.runOnce(drive::stop),

        shooter.setState(ShooterKickerState.kFeed, ShooterPivotState.kScore, ShooterRollerState.kScore)
            .alongWith(indexer.setState(IndexerState.kFeed)),

        Commands.waitSeconds(13.0)).finallyDo(() -> {
          intake.setIdle();
          indexer.setIdle();
          shooter.setIdle();
        });
  }

  private Command appliedPathPlannerAuto(String autoName) {
    return new PathPlannerAuto(autoName, SmartDashboard.getBoolean("Auto/MirrorPath", false));
  }
}
