package frc.robot.selfdriving;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Allocated;

public abstract class GameElementLocation {
    public enum Quadrant { // locations are from top-down view of the alliance with the stations at the bottom
        // Bottom right
        Q1, 
        // Bottom left
        Q2, 
        // Top left
        Q3, 
        // Top right
        Q4;

        private static Pose2d q1Middle = new Pose2d(FieldConstants.fieldLength / 8, FieldConstants.fieldWidth * (3 / 4), Rotation2d.kZero);
        private static Pose2d q2Middle = new Pose2d(FieldConstants.fieldLength / 8, FieldConstants.fieldWidth / 4, Rotation2d.kZero);
        private static Pose2d q3Middle = new Pose2d(FieldConstants.fieldLength * (3 / 8), FieldConstants.fieldWidth / 4, Rotation2d.kZero);
        private static Pose2d q4Middle = new Pose2d(FieldConstants.fieldLength * (3 / 8), FieldConstants.fieldWidth * (3 / 4), Rotation2d.kZero);

        public static Quadrant fromPoint(Pose2d pose) {
            pose = AllianceFlipUtil.apply(pose);

            double x = pose.getX(), y = pose.getY();
            if (y > FieldConstants.fieldWidth / 2) {
                // Q2 or Q3
                return x > FieldConstants.fieldLength * (3 / 8) ? Q3 : Q2;
            } else {
                // Q1 or Q4
                return x > FieldConstants.fieldLength * (3 / 8) ? Q4 : Q1;
            }
        }
        public Pose2d middle() {
            switch (this) {
                case Q1: return q1Middle;
                case Q2: return q2Middle;
                case Q3: return q3Middle;
                case Q4: return q4Middle;
                default: throw new Error("Unknown quadrant: " + this.toString());
            }
        }
    }
    public enum GamePiece {
        kCoral,
        kAlgae,
    }

    public class ReefPole {
        public Pose2d getPose() {
            return new Pose2d();
        }
    }

    public static class CoralStation extends GameElementLocation {
        private final Allocated<Boolean> hasMoreCoral;

        public CoralStation(Pose2d location, Allocated<Boolean> hasCoralMarker) {
            super(location, GamePiece.kCoral);
            hasMoreCoral = hasCoralMarker;
        }

        @Override
        public boolean getAvailability() {
            return hasMoreCoral.get();
        }
    }

    public static class LeftCoralStation {
        private static final Allocated<Boolean> leftHasCoral = new Allocated<Boolean>(true);

        public static GameElementLocation closeFace = new CoralStation(
                AllianceFlipUtil.apply(new Pose2d(0.7, 6.705, new Rotation2d(Units.degreesToRadians(-54)))),
                leftHasCoral);
        public static GameElementLocation centerFace = new CoralStation(
                AllianceFlipUtil.apply(new Pose2d(1.1655, 7.0525, new Rotation2d(Units.degreesToRadians(-54)))),
                leftHasCoral);
        public static GameElementLocation farFace = new CoralStation(
                AllianceFlipUtil.apply(new Pose2d(1.633, 7.4, new Rotation2d(Units.degreesToRadians(-54)))),
                leftHasCoral);
    }

    public static class RightCoralStation {
        private static final Allocated<Boolean> rightHasCoral = new Allocated<Boolean>(true);

        public final static GameElementLocation closeFace = new CoralStation(
                AllianceFlipUtil.apply(new Pose2d(0.7, 1.325, new Rotation2d(Units.degreesToRadians(54)))),
                rightHasCoral);
        public final static GameElementLocation centerFace = new CoralStation(
                AllianceFlipUtil.apply(new Pose2d(1.1655, 0.9975, new Rotation2d(Units.degreesToRadians(54)))),
                rightHasCoral);
        public final static GameElementLocation farFace = new CoralStation(
                AllianceFlipUtil.apply(new Pose2d(1.1655, 0.9975, new Rotation2d(Units.degreesToRadians(54)))),
                rightHasCoral);
    }

    private final Pose2d location;
    private final GamePiece type;

    public GameElementLocation(Pose2d location, GamePiece type) {
        this.location = location;
        this.type = type;
    }

    public abstract boolean getAvailability();

    public GamePiece getType() { 
        return type;
    } 
}
