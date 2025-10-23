// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage

package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;

public class AllianceFlipUtil {
  public static double applyXUnchecked(double x) {
    return FieldConstants.fieldLength - x;
  }

  public static double applyX(double x) {
    return shouldFlip() ? FieldConstants.fieldLength - x : x;
  }

  public static double applyYUnchecked(double y) {
    return FieldConstants.fieldWidth - y;
  }

  public static double applyY(double y) {
    return shouldFlip() ? FieldConstants.fieldWidth - y : y;
  }

  public static Translation2d applyUnchecked(Translation2d translation) {
    return new Translation2d(
        applyXUnchecked(translation.getX()), applyYUnchecked(translation.getX()));
  }

  public static Translation2d apply(Translation2d translation) {
    return shouldFlip() ? applyUnchecked(translation) : translation;
  }

  public static Rotation2d applyUnchecked(Rotation2d rotation) {
    return rotation.rotateBy(Rotation2d.k180deg);
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? applyUnchecked(rotation) : rotation;
  }

  public static Pose2d applyUnchecked(Pose2d pose) {
    return new Pose2d(applyUnchecked(pose.getTranslation()), applyUnchecked(pose.getRotation()));
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip() ? applyUnchecked(pose) : pose;
  }

  public static Transform2d applyUnchecked(Transform2d transform) {
    return new Transform2d(-transform.getX(), -transform.getY(), transform.getRotation().times(-1));
  }

  public static Transform2d apply(Transform2d transform) {
    return shouldFlip() ? applyUnchecked(transform) : transform;
  }

  public static Translation3d apply(Translation3d translation) {
    return new Translation3d(
        applyX(translation.getX()), applyY(translation.getY()), translation.getZ());
  }

  public static Rotation3d apply(Rotation3d rotation) {
    return shouldFlip() ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
  }

  public static Pose3d apply(Pose3d pose) {
    return new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()));
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }
}
