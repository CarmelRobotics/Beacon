package frc.robot.util;

public record CameraInfo(
    String name,
    double forward,
    double side,
    double up,
    double roll,
    double pitch,
    double yaw,
    double stdDev
) {}