// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;

public class Platform {
  public static final boolean isDesktop = new File("build.gradle").exists();
  public static final boolean isRobot = !isDesktop;

  public static File getDeployDirectory() {
    if (isRobot) {
      return new File(Filesystem.getOperatingDirectory(), "deploy");
    } else {
      return new File(
          Filesystem.getOperatingDirectory(),
          "src" + File.separator + "main" + File.separator + "deploy");
    }
  }
}