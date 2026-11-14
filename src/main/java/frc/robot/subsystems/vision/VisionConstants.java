package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.CameraInfo;

public class VisionConstants {
  public static final AprilTagFieldLayout kAprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final CameraInfo kLimelightLeft = new CameraInfo(
      "limelight-left",
      new Transform3d(
          Meters.of(-0.266),
          Meters.of(-0.14605),
          Meters.of(0.2),
          new Rotation3d(
              Degrees.zero(),
              Degrees.of(9.7),
              Degrees.of(-159.8))),
      1.0);
  public static final CameraInfo kLimelightRight = new CameraInfo(
      "limelight-right",
      new Transform3d(
          Meters.of(-0.266),
          Meters.of(0.14605),
          Meters.of(0.2),
          new Rotation3d(
              Degrees.zero(),
              Degrees.of(9.7),
              Degrees.of(159.8))),
      1.0);

  public static final CameraInfo[] kCameraInfo = new CameraInfo[] {
      kLimelightLeft,
      kLimelightRight
  };

  public static final double kMaxAmbiguity = 0.3;
  public static final double kMaXZError = 0.75;

  public static final double kLinearStdDevBaseline = 0.02;
  public static final double kAngularStdDevBaseline = 0.06;

  public static final double kLinearStdDevMegatag2Factor = 0.5;
  public static final double kAngularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}
