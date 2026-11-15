// Copyright (c) 2025-2026 Rockin' Bots
// https://github.com/CarmelRobotics

package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.kicker.ShooterKicker;
import frc.robot.subsystems.shooter.kicker.ShooterKickerIO;
import frc.robot.subsystems.shooter.pivot.ShooterPivot;
import frc.robot.subsystems.shooter.pivot.ShooterPivotIO;
import frc.robot.subsystems.shooter.roller.ShooterRoller;
import frc.robot.subsystems.shooter.roller.ShooterRollerIO;
import frc.robot.util.geometry.AllianceFlip;
import frc.robot.util.geometry.FieldConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterKickerState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterPivotState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterRollerState;

public class Shooter {
  private final ShooterPivot pivot;
  private final ShooterRoller roller;
  private final ShooterKicker kicker;

  private final Supplier<Pose2d> pose;

  public Shooter(
      ShooterKickerIO kickerio,
      ShooterPivotIO pivotio,
      ShooterRollerIO rollerio,
      Supplier<Pose2d> pose,
      BooleanSupplier facingTarget) {
    this.pose = pose;
    this.pivot = new ShooterPivot(pivotio);
    this.roller = new ShooterRoller(rollerio, facingTarget, this::getScoreDistance);
    this.kicker = new ShooterKicker(kickerio, roller::isReadyToFeed);
  }

  public Command setState(ShooterKickerState state) {
    return kicker.setState(state);
  }

  public Command setState(ShooterPivotState state) {
    return pivot.setState(state);
  }

  public Command setState(ShooterRollerState state) {
    return roller.setState(state);
  }

  public void setIdle() {
    kicker.setIdle();
    roller.setIdle();
    pivot.setIdle();
  }

  public Command setState(ShooterKickerState kickerState, ShooterPivotState pivotState,
      ShooterRollerState rollerState) {
    return kicker.setState(kickerState).alongWith(pivot.setState(pivotState)).alongWith(roller.setState(rollerState));
  }

  public ShooterKickerState getKickerState() {
    return kicker.getState();
  }

  public ShooterPivotState getPivotState() {
    return pivot.getState();
  }

  public ShooterRollerState getRollerState() {
    return roller.getState();
  }

  @AutoLogOutput(key = "Shooter/DistanceToScore")
  public double getScoreDistance() {
    return pose.get().getTranslation()
        .getDistance(AllianceFlip.apply(FieldConstants.Hub.kTopCenterPoint).toTranslation2d());
  }

  @AutoLogOutput(key = "Shooter/DistanceToPass")
  public double getPassDistance() {
    double zoneLineX = AllianceFlip.apply(
        new Translation2d(FieldConstants.LinesVertical.kAllianceZone, 0.0)).getX();
    return Math.abs(pose.get().getX() - zoneLineX);
  }

  public boolean isReadyToFeed() {
    return roller.isReadyToFeed();
  }

  public Command zeroPivotHere() {
    return pivot.zeroHere();
  }
}
