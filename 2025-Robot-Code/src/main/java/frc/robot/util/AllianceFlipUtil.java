// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage

package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;


public class AllianceFlipUtil {

  public static double applyX(double x) {
    return shouldFlip() ? FieldConstants.kFieldLength - x : x;
  }

  public static double applyY(double y) {
    return shouldFlip() ? FieldConstants.kFieldWidth - y : y;
  }

  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  public static double applyXUnchecked(double x) {
    return FieldConstants.kFieldLength - x;
  }
  public static double applyYUnchecked(double y) {
    return FieldConstants.kFieldWidth - y;
  }
  public static Translation2d applyUnchecked(Translation2d translation) {
    return new Translation2d(applyXUnchecked(translation.getX()), applyYUnchecked(translation.getY()));
  }
  public static Rotation2d applyUnchecked(Rotation2d rotation) {
    return rotation.rotateBy(Rotation2d.kPi);
  }
  public static Pose2d applyUnchecked(Pose2d pose) {
    return new Pose2d(applyUnchecked(pose.getTranslation()), applyUnchecked(pose.getRotation()));
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
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }
}