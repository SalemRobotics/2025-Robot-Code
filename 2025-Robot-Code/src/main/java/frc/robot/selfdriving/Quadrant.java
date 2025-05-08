package frc.robot.selfdriving;

import static frc.robot.FieldConstants.fieldLength;
import static frc.robot.FieldConstants.fieldWidth;
import static frc.robot.Constants.AutoConstants;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.util.AllianceFlipUtil;

public enum Quadrant { // locations are from top-down view of the alliance with the stations at the
    // bottom
    // Bottom left
    Q1,
    // Bottom right
    Q2,
    // Top right
    Q3,
    // Top left
    Q4;

    // paths from Q1 -> Q3
    public static PathPlannerPath q1ToQ3_bottom, q1ToQ3_top;
    // paths from Q2 -> Q4
    public static PathPlannerPath q2ToQ4_bottom, q2ToQ4_top;
    // paths from Q3 -> Q1
    public static PathPlannerPath q3ToQ1_bottom, q3ToQ1_top;
    // paths from Q4 -> Q2
    public static PathPlannerPath q4ToQ2_bottom, q4ToQ2_top;

    static {
        try {
            q1ToQ3_bottom = PathPlannerPath.fromPathFile("Q1 to Q3 bottom");
            q1ToQ3_top = PathPlannerPath.fromPathFile("Q1 to Q3 top");

            q2ToQ4_bottom = PathPlannerPath.fromPathFile("Q2 to Q4 bottom");
            q2ToQ4_top = PathPlannerPath.fromPathFile("Q2 to Q4 top");

            q3ToQ1_bottom = PathPlannerPath.fromPathFile("Q3 to Q1 bottom");
            q3ToQ1_top = PathPlannerPath.fromPathFile("Q3 to Q1 top");

            q4ToQ2_bottom = PathPlannerPath.fromPathFile("Q2 to Q4 bottom");
            q4ToQ2_top = PathPlannerPath.fromPathFile("Q2 to Q4 top");

            if (AllianceFlipUtil.shouldFlip()) {
                q1ToQ3_bottom = q1ToQ3_bottom.flipPath();
                q1ToQ3_top = q1ToQ3_top.flipPath();

                q2ToQ4_bottom = q2ToQ4_bottom.flipPath();
                q2ToQ4_top = q2ToQ4_top.flipPath();

                q3ToQ1_bottom = q3ToQ1_bottom.flipPath();
                q3ToQ1_top = q3ToQ1_top.flipPath();

                q4ToQ2_bottom = q4ToQ2_bottom.flipPath();
                q4ToQ2_top = q4ToQ2_top.flipPath();
            }
        } catch (Exception e) {
            System.err.println("\n\n" + "=".repeat(20) + "\n\n" + e.getMessage() + e.getStackTrace() + "\n\n"
                    + "=".repeat(20) + "\n\n");
        }
    }

    /** The middle of Quadrant 1 */
    private static final Pose2d q1Middle = AllianceFlipUtil
            .apply(new Pose2d(fieldLength / 8, fieldWidth * (3 / 4), new Rotation2d()));
    /** The middle of Quadrant 2 */
    private static final Pose2d q2Middle = AllianceFlipUtil
            .apply(new Pose2d(fieldLength / 8, fieldWidth / 4, new Rotation2d()));
    /** The middle of Quadrant 3 */
    private static final Pose2d q3Middle = AllianceFlipUtil
            .apply(new Pose2d(fieldLength * (3 / 8), fieldWidth / 4, new Rotation2d()));
    /** The middle of Quadrant 4 */
    private static final Pose2d q4Middle = AllianceFlipUtil
            .apply(new Pose2d(fieldLength * (3 / 8), fieldWidth * (3 / 4), new Rotation2d()));

    private static final List<Pose2d> middles = List.of(q1Middle, q2Middle, q3Middle, q4Middle);

    public boolean isDiagonal(Quadrant other) {
        return (this == Q1 && other == Q2) || (this == Q2 && other == Q4) || (this == Q3 && other == Q1)
                || (this == Q4 && other == Q2);
    }

    public static Pose2d nearestMiddle(Pose2d pose) {
        return pose.nearest(middles);
    }

    public static Quadrant fromPose(Pose2d pose) {
        if (pose.getY() > fieldWidth / 2) {
            return pose.getX() < fieldLength / 4 ? Q2 : Q3;
        } else {
            return pose.getX() < fieldLength / 4 ? Q1 : Q4;
        }
    }

    public Pose2d middle() {
        switch (this) {
            case Q1:
                return q1Middle;
            case Q2:
                return q2Middle;
            case Q3:
                return q3Middle;
            case Q4:
                return q4Middle;
            default:
                throw new Error("Unknown quadrant " + this);
        }
    }

    public Command driveToOther(Quadrant other, boolean preferTop) {
        switch (this) {
            case Q1:
                switch (other) {
                    case Q1:
                        return Commands.none();
                    case Q2:
                        return AutoBuilder.pathfindToPose(q2Middle, AutoConstants.kPathConstraints);
                    case Q3:
                        return AutoBuilder.followPath(preferTop ? q1ToQ3_top : q1ToQ3_bottom);
                    case Q4:
                        return AutoBuilder.pathfindToPose(q4Middle, AutoConstants.kPathConstraints);
                    default:
                        throw new Error(
                                "Unknown other quadrant " + other + " (Finding path at quadrant " + this + ") [1]");
                }
            case Q2:
                switch (other) {
                    case Q1:
                        return AutoBuilder.pathfindToPose(q1Middle, AutoConstants.kPathConstraints);
                    case Q2:
                        return Commands.none();
                    case Q3:
                        return AutoBuilder.pathfindToPose(q3Middle, AutoConstants.kPathConstraints);
                    case Q4:
                        return AutoBuilder.followPath(preferTop ? q2ToQ4_top : q2ToQ4_bottom);
                    default:
                        throw new Error(
                                "Unknown other quadrant " + other + " (Finding path at quadrant " + this + ") [2]");
                }
            case Q3:
                switch (other) {
                    case Q1:
                        return AutoBuilder.followPath(preferTop ? q3ToQ1_top : q3ToQ1_bottom);
                    case Q2:
                        return AutoBuilder.pathfindToPose(q2Middle, AutoConstants.kPathConstraints);
                    case Q3:
                        return Commands.none();
                    case Q4:
                        return AutoBuilder.pathfindToPose(q4Middle, AutoConstants.kPathConstraints);
                    default:
                        throw new Error(
                                "Unknown other quadrant " + other + " (Finding path at quadrant " + this + ") [3]");
                }
            case Q4:
                switch (other) {
                    case Q1:
                        return AutoBuilder.pathfindToPose(q1Middle, AutoConstants.kPathConstraints);
                    case Q2:
                        return AutoBuilder.followPath(preferTop ? q4ToQ2_top : q4ToQ2_bottom);
                    case Q3:
                        return AutoBuilder.pathfindToPose(q3Middle, AutoConstants.kPathConstraints);
                    case Q4:
                        Commands.none();
                    default:
                        throw new Error(
                                "Unknown other quadrant " + other + " (Finding path at quadrant " + this + ") [4]");
                }
            default:
                throw new Error("Trying to pathfind from unknown quadrant " + this);
        }
    }
}
