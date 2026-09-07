package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.util.CameraInfo;

public class VisionConstants {
  public static final AprilTagFieldLayout kAprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final CameraInfo[] kCameraInfo = 
    new CameraInfo[] {
      new CameraInfo("limelight-left", 0, 0, 0, 0, 0, 0, 1.0),
      new CameraInfo("limelight-right", 0, 0, 0, 0, 0, 0, 1.0)
    };

  public static final double kMaxAmbiguity = 0.3;
  public static final double kMaXZError = 0.75;

  public static final double kLinearStdDevBaseline = 0.02;
  public static final double kAngularStdDevBaseline = 0.06;
  
  public static final double kLinearStdDevMegatag2Factor = 0.5;
  public static final double kAngularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
