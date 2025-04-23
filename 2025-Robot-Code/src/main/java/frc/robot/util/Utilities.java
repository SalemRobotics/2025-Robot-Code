package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class Utilities {
    public static double getDistance(Pose2d a, Pose2d b) {
        return a.getTranslation().getDistance(b.getTranslation());
    }
}
