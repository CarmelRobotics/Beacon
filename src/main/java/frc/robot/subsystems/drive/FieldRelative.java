package frc.robot.subsystems.drive;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.geometry.AllianceFlip;
import frc.robot.util.geometry.Bounds;
import frc.robot.util.geometry.FieldConstants;

public class FieldRelative {
  private final Supplier<Pose2d> pose;

  public FieldRelative(Supplier<Pose2d> pose) {
    this.pose = pose;
  }

  public boolean isInAllianceZone() {
    return FieldConstants.kAllianceZone.contains(AllianceFlip.apply(pose.get().getTranslation()));
  }

  public Translation2d hubTarget() {
    return AllianceFlip.apply(FieldConstants.Hub.kInnerCenterPoint.toTranslation2d());
  }

  public Optional<Translation2d> passingTarget() {
    if (isInAllianceZone()) {
      return Optional.empty();
    }

    final Translation2d robot = AllianceFlip.apply(pose.get().getTranslation());
    Translation2d best = null;
    double bestDistance = Double.POSITIVE_INFINITY;

    for (Translation2d target : FieldConstants.kPassingTargets) {
      double distance = robot.getDistance(target);
      if (distance > bestDistance) {
        continue;
      }

      boolean blocked = false;
      for (Bounds obstruction : FieldConstants.kObstructions) {
        if (obstruction.contains(robot)) {
          continue;
        } else if (blocks(obstruction, robot, target)) {
          blocked = true;
          break;
        }
      }
      if (blocked) {
        continue;
      }

      best = target;
      bestDistance = distance;
    }

    return Optional.ofNullable(best).map(t -> AllianceFlip.apply(t));
  }

  // Whether passing from the robot to the target would be blocked by the obstruction
  private static boolean blocks(Bounds obstruction, Translation2d robot, Translation2d target) {
    double[] lows = {obstruction.minX(), obstruction.minY()};
    double[] highs = {obstruction.maxX(), obstruction.maxY()};
    double[] position = {robot.getX(), robot.getY()};
    double[] direction = {target.getX() - robot.getX(), target.getY() - robot.getY()};

    double pathMin = 0.0;
    double pathMax = 1.0;
    for (int i = 0; i < 2; i++) {
      if (Math.abs(direction[i]) < 1e-9) {
        if (position[i] < lows[i] || position[i] > highs[i]) {
          return false;
        }
      } else {
        double pathLow = (lows[i] - position[i]) / direction[i];
        double pathHigh = (highs[i] - position[i]) / direction[i];
        pathMin = Math.max(pathMin, Math.min(pathLow, pathHigh));
        pathMax = Math.min(pathMax, Math.max(pathLow, pathHigh));

        if (pathLow > pathMax) {
          return false;
        }
      }
    }

    return true;
  }
}
