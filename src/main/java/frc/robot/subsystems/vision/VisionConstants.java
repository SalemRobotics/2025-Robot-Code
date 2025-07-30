// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static final String camera0Name = "Left";
  public static final String camera1Name = "Right";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static final Transform3d robotToCamera0 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(11.58),
              Units.inchesToMeters(11.189),
              Units.inchesToMeters(8.25)),
          new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-20)));
  public static final Transform3d robotToCamera1 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(11.58),
              Units.inchesToMeters(-11.189),
              Units.inchesToMeters(8.25)),
          new Rotation3d(0, Units.degreesToRadians(-15.0), Units.degreesToRadians(15.0)));

  // Basic filtering thresholds
  public static final double maxAmbiguity = 0.3;
  public static final double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final double linearStdDevBaseline = 0.02; // Meters
  public static final double angularStdDevBaseline = 0.2; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static final double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static final double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static final double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  public static final int numCameras = 2;
}
