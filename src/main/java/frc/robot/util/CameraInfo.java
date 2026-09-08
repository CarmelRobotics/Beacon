package frc.robot.util;

import edu.wpi.first.math.geometry.Transform3d;

public record CameraInfo(
    String name,
    Transform3d robotToCamera,
    double stdDev
) {}