// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.geometry;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.Platform;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Contains information for location of field element and other useful reference
 * points.
 *
 * <p>
 * NOTE: All constants are defined relative to the field coordinate system, and
 * from the
 * perspective of the blue alliance station
 */
public class FieldConstants {
  public static final FieldType kFieldType = FieldType.kAndyMark;

  // AprilTag related constants
  public static final int kAprilTagCount = AprilTagLayoutType.kOfficial.getLayout().getTags().size();
  public static final double kAprilTagWidth = Units.inchesToMeters(6.5);
  public static final AprilTagLayoutType kDefaultAprilTagType = AprilTagLayoutType.kOfficial;

  // Field dimensions
  public static final double kFieldLength = AprilTagLayoutType.kOfficial.getLayout().getFieldLength();
  public static final double kFieldWidth = AprilTagLayoutType.kOfficial.getLayout().getFieldWidth();
  public static final Translation2d kFieldCenter = new Translation2d(kFieldLength / 2.0, kFieldWidth / 2.0);

  // Fuel dimensions
  public static final double kFuelDiameter = Units.inchesToMeters(5.91);

  public static final double kTrenchBarWidth = Units.inchesToMeters(2.95);

  /**
   * Officially defined and relevant vertical lines found on the field (defined by
   * X-axis offset)
   */
  public static class LinesVertical {
    public static final double kCenter = kFieldLength / 2.0;
    public static final double kStarting = AprilTagLayoutType.kOfficial.getLayout().getTagPose(26).get().getX();
    public static final double kAllianceZone = kStarting;
    public static final double kHubCenter = AprilTagLayoutType.kOfficial.getLayout().getTagPose(26).get().getX()
        + Hub.kWidth / 2.0;
    public static final double kNeutralZoneNear = kCenter - Units.inchesToMeters(120);
    public static final double kNeutralZoneFar = kCenter + Units.inchesToMeters(120);
    public static final double kOppHubCenter = AprilTagLayoutType.kOfficial.getLayout().getTagPose(4).get().getX()
        + Hub.kWidth / 2.0;
    public static final double kOppAllianceZone = AprilTagLayoutType.kOfficial.getLayout().getTagPose(10).get().getX();
  }

  /**
   * Officially defined and relevant horizontal lines found on the field (defined
   * by Y-axis offset)
   *
   * <p>
   * NOTE: The field element start and end are always left to right from the
   * perspective of the
   * alliance station
   */
  public static class LinesHorizontal {

    public static final double kCenter = kFieldWidth / 2.0;

    // Right of hub
    public static final double kRightBumpStart = Hub.kNearRightCorner.getY();
    public static final double kRightBumpEnd = kRightBumpStart - RightBump.kWidth;
    public static final double kRightBumpMiddle = (kRightBumpStart + kRightBumpEnd) / 2.0;
    public static final double kRightTrenchOpenStart = kRightBumpEnd - Units.inchesToMeters(12.0);
    public static final double kRightTrenchOpenEnd = 0;
    public static final double kRightTrenchMiddle = (kRightTrenchOpenStart + kRightTrenchOpenEnd) / 2.0;

    // Left of hub
    public static final double kLeftBumpEnd = Hub.kNearLeftCorner.getY();
    public static final double kLeftBumpStart = kLeftBumpEnd + LeftBump.kWidth;
    public static final double kLeftBumpMiddle = (kLeftBumpStart + kLeftBumpEnd) / 2.0;
    public static final double kLeftTrenchOpenEnd = kLeftBumpStart + Units.inchesToMeters(12.0);
    public static final double kLeftTrenchOpenStart = kFieldWidth;
    public static final double kLeftTrenchMiddle = (kLeftTrenchOpenEnd + kLeftTrenchOpenStart) / 2.0;
  }

  /** Hub related constants */
  public static class Hub {

    // Dimensions
    public static final double kWidth = Units.inchesToMeters(47.0);
    public static final double kHeight = Units.inchesToMeters(72.0); // includes the catcher at the top
    public static final double kInnerWidth = Units.inchesToMeters(41.7);
    public static final double kInnerHeight = Units.inchesToMeters(56.5);

    // Relevant reference points on alliance side
    public static final Translation3d kTopCenterPoint = new Translation3d(
        AprilTagLayoutType.kOfficial.getLayout().getTagPose(26).get().getX() + kWidth / 2.0,
        kFieldWidth / 2.0,
        kHeight);
    public static final Translation3d kInnerCenterPoint = new Translation3d(
        AprilTagLayoutType.kOfficial.getLayout().getTagPose(26).get().getX() + kWidth / 2.0,
        kFieldWidth / 2.0,
        kInnerHeight);

    public static final Translation2d kNearLeftCorner = new Translation2d(kTopCenterPoint.getX() - kWidth / 2.0,
        kFieldWidth / 2.0 + kWidth / 2.0);
    public static final Translation2d kNearRightCorner = new Translation2d(kTopCenterPoint.getX() - kWidth / 2.0,
        kFieldWidth / 2.0 - kWidth / 2.0);
    public static final Translation2d kFarLeftCorner = new Translation2d(kTopCenterPoint.getX() + kWidth / 2.0,
        kFieldWidth / 2.0 + kWidth / 2.0);
    public static final Translation2d kFarRightCorner = new Translation2d(kTopCenterPoint.getX() + kWidth / 2.0,
        kFieldWidth / 2.0 - kWidth / 2.0);

    // Hub faces
    public static final Pose2d kNearFace = AprilTagLayoutType.kOfficial.getLayout().getTagPose(26).get().toPose2d();
    public static final Pose2d kFarFace = AprilTagLayoutType.kOfficial.getLayout().getTagPose(20).get().toPose2d();
    public static final Pose2d kRightFace = AprilTagLayoutType.kOfficial.getLayout().getTagPose(18).get().toPose2d();
    public static final Pose2d kLeftFace = AprilTagLayoutType.kOfficial.getLayout().getTagPose(21).get().toPose2d();
  }

  /** Left Bump related constants */
  public static class LeftBump {

    // Dimensions
    public static final double kWidth = Units.inchesToMeters(73.0);
    public static final double kHeight = Units.inchesToMeters(6.513);
    public static final double kDepth = Units.inchesToMeters(44.4);

    // Relevant reference points on alliance side
    public static final Translation2d kNearLeftCorner = Hub.kNearLeftCorner.plus(new Translation2d(0.0, kWidth));
    public static final Translation2d kNearRightCorner = Hub.kNearLeftCorner;
    public static final Translation2d kFarLeftCorner = Hub.kFarLeftCorner.plus(new Translation2d(0.0, kWidth));
    public static final Translation2d kFarRightCorner = Hub.kFarLeftCorner;
  }

  /** Right Bump related constants */
  public static class RightBump {
    // Dimensions
    public static final double kWidth = Units.inchesToMeters(73.0);
    public static final double kHeight = Units.inchesToMeters(6.513);
    public static final double kDepth = Units.inchesToMeters(44.4);

    // Relevant reference points on alliance side
    public static final Translation2d kNearLeftCorner = Hub.kNearRightCorner;
    public static final Translation2d kNearRightCorner = Hub.kNearRightCorner.minus(new Translation2d(0.0, kWidth));
    public static final Translation2d kFarLeftCorner = Hub.kFarRightCorner;
    public static final Translation2d kFarRightCorner = Hub.kFarRightCorner.minus(new Translation2d(0.0, kWidth));
  }

  /** Left Trench related constants */
  public static class LeftTrench {
    // Dimensions
    public static final double kWidth = Units.inchesToMeters(65.65);
    public static final double kDepth = Units.inchesToMeters(47.0);
    public static final double kHeight = Units.inchesToMeters(40.25);
    public static final double kOpeningWidth = Units.inchesToMeters(50.34);
    public static final double kOpeningHeight = Units.inchesToMeters(22.25);

    // Relevant reference points on alliance side
    public static final Translation3d kOpeningTopLeft = new Translation3d(LinesVertical.kHubCenter, kFieldWidth,
        kOpeningHeight);
    public static final Translation3d kOpeningTopRight = new Translation3d(LinesVertical.kHubCenter,
        kFieldWidth - kOpeningWidth, kOpeningHeight);
    public static final Translation2d kCenter = kOpeningTopLeft.toTranslation2d()
        .interpolate(kOpeningTopRight.toTranslation2d(), 0.5);
  }

  public static class RightTrench {

    // Dimensions
    public static final double kWidth = Units.inchesToMeters(65.65);
    public static final double kDepth = Units.inchesToMeters(47.0);
    public static final double kHeight = Units.inchesToMeters(40.25);
    public static final double kOpeningWidth = Units.inchesToMeters(50.34);
    public static final double kOpeningHeight = Units.inchesToMeters(22.25);

    // Relevant reference points on alliance side
    public static final Translation3d kOpeningTopLeft = new Translation3d(LinesVertical.kHubCenter, kOpeningWidth,
        kOpeningHeight);
    public static final Translation3d kOpeningTopRight = new Translation3d(LinesVertical.kHubCenter, 0, kOpeningHeight);
    public static final Translation2d kCenter = kOpeningTopLeft.toTranslation2d()
        .interpolate(kOpeningTopRight.toTranslation2d(), 0.5);
  }

  /** Tower related constants */
  public static class Tower {
    // Dimensions
    public static final double kWidth = Units.inchesToMeters(49.25);
    public static final double kDepth = Units.inchesToMeters(45.0);
    public static final double kHeight = Units.inchesToMeters(78.25);
    public static final double kInnerOpeningWidth = Units.inchesToMeters(32.250);
    public static final double kFrontFaceX = Units.inchesToMeters(43.51);

    public static final double kUprightHeight = Units.inchesToMeters(72.1);

    // Rung heights from the floor
    public static final double kLowRungHeight = Units.inchesToMeters(27.0);
    public static final double kMidRungHeight = Units.inchesToMeters(45.0);
    public static final double kHighRungHeight = Units.inchesToMeters(63.0);

    // Relevant reference points on alliance side
    public static final Translation2d kCenterPoint = new Translation2d(
        kFrontFaceX, AprilTagLayoutType.kOfficial.getLayout().getTagPose(31).get().getY());
    public static final Translation2d kLeftUpright = new Translation2d(
        kFrontFaceX,
        (AprilTagLayoutType.kOfficial.getLayout().getTagPose(31).get().getY())
            + kInnerOpeningWidth / 2
            + Units.inchesToMeters(0.75));
    public static final Translation2d kRightUpright = new Translation2d(
        kFrontFaceX,
        (AprilTagLayoutType.kOfficial.getLayout().getTagPose(31).get().getY())
            - kInnerOpeningWidth / 2
            - Units.inchesToMeters(0.75));
  }

  public static class Depot {
    // Dimensions
    public static final double kWidth = Units.inchesToMeters(42.0);
    public static final double kDepth = Units.inchesToMeters(27.0);
    public static final double kHeight = Units.inchesToMeters(1.125);
    public static final double kDistanceFromCenterY = Units.inchesToMeters(75.93);

    // Relevant reference points on alliance side
    public static final Translation3d kDepotCenter = new Translation3d(kDepth, (kFieldWidth / 2) + kDistanceFromCenterY,
        kHeight);
    public static final Translation3d kLeftCorner = new Translation3d(kDepth,
        (kFieldWidth / 2) + kDistanceFromCenterY + (kWidth / 2), kHeight);
    public static final Translation3d kRightCorner = new Translation3d(kDepth,
        (kFieldWidth / 2) + kDistanceFromCenterY - (kWidth / 2), kHeight);
  }

  public static class Outpost {
    // Dimensions
    public static final double kWidth = Units.inchesToMeters(31.8);
    public static final double kOpeningDistanceFromFloor = Units.inchesToMeters(28.1);
    public static final double kHeight = Units.inchesToMeters(7.0);

    // Relevant reference points on alliance side
    public static final Translation2d kCenterPoint = new Translation2d(0,
        AprilTagLayoutType.kOfficial.getLayout().getTagPose(29).get().getY());
  }

  public static class FuelPool {
    // Dimensions
    public static final double kWidth = Units.inchesToMeters(181.9);
    public static final double kDepth = Units.inchesToMeters(71.9);

    // Relevant reference points on alliance side
    public static final Translation2d kNearLeftCorner = new Translation2d(kFieldLength / 2.0 - kDepth / 2.0,
        kFieldWidth / 2.0 + kWidth / 2.0);
    public static final Translation2d kNearRightCorner = new Translation2d(kFieldLength / 2.0 - kDepth / 2.0,
        kFieldWidth / 2.0 - kWidth / 2.0);
    public static final Translation2d kLeftCenter = new Translation2d(kFieldLength / 2.0, kFieldWidth / 2.0 + kWidth / 2.0);
    public static final Translation2d kRightCenter = new Translation2d(kFieldLength / 2.0,
        kFieldWidth / 2.0 - kWidth / 2.0);
  }

  public static final Bounds kAllianceZone =
    new Bounds(0.0, LinesVertical.kHubCenter, 0.0, kFieldWidth);

  public static final List<Bounds> kObstructions = List.of(
    new Bounds(
      Hub.kNearRightCorner.getX(), Hub.kFarLeftCorner.getX(),
      Hub.kNearRightCorner.getY(), Hub.kFarLeftCorner.getY()),
    new Bounds(
      0.0, Tower.kDepth,
      Tower.kCenterPoint.getY() - (Tower.kWidth / 2.0),
      Tower.kCenterPoint.getY() + (Tower.kWidth / 2.0)),
    new Bounds(
      LinesVertical.kHubCenter - (LeftTrench.kDepth / 2.0),
      LinesVertical.kHubCenter + (LeftTrench.kDepth / 2.0),
      kFieldWidth - LeftTrench.kWidth,
      kFieldWidth),
    new Bounds(
      LinesVertical.kHubCenter - (RightTrench.kDepth / 2.0),
      LinesVertical.kHubCenter + (RightTrench.kDepth / 2.0),
      0.0,
      RightTrench.kWidth)
  );

  public static final double kPassTargetX = LinesVertical.kAllianceZone / 2.0;

  public static final List<Translation2d> kPassingTargets = List.of(
    new Translation2d(kPassTargetX, LinesHorizontal.kLeftTrenchMiddle),
    new Translation2d(kPassTargetX, LinesHorizontal.kLeftBumpMiddle),
    new Translation2d(kPassTargetX, LinesHorizontal.kRightBumpMiddle),
    new Translation2d(kPassTargetX, LinesHorizontal.kRightTrenchMiddle)
  );

  @RequiredArgsConstructor
  public enum FieldType {
    kHQ("hq"),
    kAndyMark("andymark"),
    kWelded("welded");

    @Getter
    private final String jsonFolder;
  }

  public enum AprilTagLayoutType {
    kOfficial("2026-official"),
    kNone("2026-none"),
    kHub("2026-hub"),
    kOutpost("2026-outpost"),
    kTower("2026-tower");

    private final String name;
    private volatile AprilTagFieldLayout layout;
    private volatile String layoutString;

    AprilTagLayoutType(String name) {
      this.name = name;
    }

    public AprilTagFieldLayout getLayout() {
      if (layout == null) {
        synchronized (this) {
          if (layout == null) {
            try {
              Path p = Constants.kDisableHAL
                  ? Path.of(
                      "src",
                      "main",
                      "deploy",
                      "apriltags",
                      kFieldType.getJsonFolder(),
                      name + ".json")
                  : Path.of(
                      Platform.getDeployDirectory().getPath(),
                      "apriltags",
                      kFieldType.getJsonFolder(),
                      name + ".json");
              layout = new AprilTagFieldLayout(p);
              layoutString = new ObjectMapper().writeValueAsString(layout);
            } catch (IOException e) {
              throw new RuntimeException(e);
            }
          }
        }
      }
      return layout;
    }

    public String getLayoutString() {
      if (layoutString == null) {
        getLayout();
      }
      return layoutString;
    }
  }
}