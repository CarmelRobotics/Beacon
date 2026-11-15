// Copyright (c) 2025-2026 Rockin' Bots
// https://github.com/CarmelRobotics

package frc.robot.util;

import edu.wpi.first.math.geometry.Transform3d;

public record CameraInfo(
    String name,
    Transform3d robotToCamera,
    double stdDev) {
}