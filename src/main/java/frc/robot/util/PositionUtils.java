package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.Drive;

public final class PositionUtils {
  public static double getDistance(Pose2d from, Pose2d to) {
    return from.getTranslation().getDistance(to.getTranslation());
  }

  public static double getDistance(Drive drive, Pose2d to) {
    return getDistance(drive.getPose(), to);
  }

  public static boolean isNear(Pose2d pose1, Pose2d pose2, double tolerance) {
    return MathUtil.isNear(pose1.getX(), pose2.getX(), tolerance)
        && MathUtil.isNear(pose1.getY(), pose2.getY(), tolerance)
        && MathUtil.isNear(
            pose1.getRotation().getRadians(), pose1.getRotation().getRadians(), tolerance);
  }
}
