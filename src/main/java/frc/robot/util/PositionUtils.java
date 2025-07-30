package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.Drive;

public final class PositionUtils {
  public static double getDistance(Pose2d from, Pose2d to) {
    return from.getTranslation().getDistance(to.getTranslation());
  }

  public static double getDistance(Drive drive, Pose2d to) {
    return getDistance(drive.getPose(), to);
  }
}
