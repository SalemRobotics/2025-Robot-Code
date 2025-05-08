package frc.robot.selfdriving;

import static frc.robot.util.Utilities.getDistance;

import java.util.HashMap;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.FieldConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.util.AllianceFlipUtil;

public interface SelfDriveTarget {
    public static enum GamePiece {
        Coral,
        Algae,
    }

    public static enum AllianceSide {
        Left,
        Right
    }

    public static enum CoralStationPosition {
        Left, Center, Right
    }

    public static class State {
        private static final NetworkTable stateTable = NetworkTableInstance.getDefault().getTable("scorestate");

        static class StateUpdater {
            private final NetworkTable networkTable;
            private BooleanTopic[] booleanTopics;
            private IntegerTopic[] integerTopics;
            private boolean defaultBoolean = false;
            private long defaultInt = 0;

            public StateUpdater(NetworkTable table) {
                networkTable = table;
                booleanTopics = new BooleanTopic[0];
                integerTopics = new IntegerTopic[0];
            }

            public StateUpdater withBooleanUpdates(boolean defaultValue, String... toUpdate) {
                defaultBoolean = defaultValue;
                booleanTopics = new BooleanTopic[toUpdate.length];
                for (int i = 0; i < toUpdate.length; i++) {
                    booleanTopics[i] = networkTable.getBooleanTopic(toUpdate[i]);
                }

                return this;
            }

            public StateUpdater withIntegerUpdates(long defaultValue, String... toUpdate) {
                defaultInt = defaultValue;
                integerTopics = new IntegerTopic[toUpdate.length];
                for (int i = 0; i < toUpdate.length; i++) {
                    integerTopics[i] = networkTable.getIntegerTopic(toUpdate[i]);
                }

                return this;
            }

            public boolean[] getBooleanUpdates() {
                boolean[] updated = new boolean[booleanTopics.length];

                for (int i = 0; i < booleanTopics.length; i++)
                    updated[i] = booleanTopics[i].getEntry(defaultBoolean).get();

                return updated;
            }

            public long[] getIntegerUpdates() {
                long[] updated = new long[integerTopics.length];

                for (int i = 0; i < integerTopics.length; i++)
                    updated[i] = integerTopics[i].getEntry(defaultInt).get();

                return updated;
            }
        }

        private static HashMap<Character, ReefFace> faces = new HashMap<>();
        static {
            int offset = AllianceFlipUtil.shouldFlip() ? 6 : 0;
            int i = 0;

            for (char c = 'a'; c < 'g'; c++) {
                faces.put(c, new ReefFace(
                        c,
                        FieldConstants.Reef.centerFaces[offset + i++],
                        FieldConstants.Reef.branchPositions.get(i * 2).get(ReefHeight.L4).toPose2d(),
                        FieldConstants.Reef.branchPositions.get(i * 2 + 1).get(ReefHeight.L4).toPose2d()));
            }
        }
    }

    public static class ReefPole implements SelfDriveTarget {
        private final Pose2d position;
        private final ReefFace face;
        private final NetworkTable table;

        private final boolean[] poleState = new boolean[] { false, false, false };
        private long troughState;

        private final State.StateUpdater updater;

        public ReefPole(NetworkTable parent, ReefFace reefFace, boolean isRight) {
            position = isRight ? reefFace.rightPole : reefFace.leftPole;
            face = reefFace;
            table = parent.getSubTable(isRight ? "right" : "left");
            updater = new State.StateUpdater(table)
                    .withBooleanUpdates(false, "l2", "l3", "l4")
                    .withIntegerUpdates(3, "l1");
        }

        @Override
        public Pose2d getLocation() {
            return position;
        }

        @Override
        public void updateState() {
            final boolean[] poleUpdates = updater.getBooleanUpdates();

            for (int i = 0; i < poleState.length; i++)
                poleState[i] = poleUpdates[i];

            troughState = updater.getIntegerUpdates()[0];
        }

        public ReefFace getFace() {
            return face;
        }

        public boolean getPoleAvailability(int level) {
            try {
                if (level == 1)
                    return troughState >= 0 && troughState < 3;
                else
                    return poleState[level];
            } catch (Exception e) {
                return false;
            }
        }
    }

    public static class ReefFace implements SelfDriveTarget {
        private static final ReefFace[] faces = new ReefFace[6];
        static {
            for (int i = 0; i < 6; i++)
                faces[i] = new ReefFace(
                        (char) (i + (int) 'a'),
                        FieldConstants.Reef.centerFaces[0],
                        FieldConstants.Reef.branchPositions.get(i * 2).get(ReefHeight.L1).toPose2d(),
                        FieldConstants.Reef.branchPositions.get(i * 2 + 1).get(ReefHeight.L1).toPose2d());
        }

        private final char faceId;
        private final Pose2d centerFace;
        private final Pose2d leftPole, rightPole;
        private final NetworkTable table;
        private final BooleanTopic algaeTopic;
        private boolean hasAlgae = true;

        public ReefFace(char ident, Pose2d center, Pose2d right, Pose2d left) {
            faceId = ident;
            centerFace = center;
            leftPole = left;
            rightPole = right;
            table = State.stateTable.getSubTable("reef" + ident);
            algaeTopic = table.getBooleanTopic("algae");
        }

        public ReefPole getLeftPole() {
            return new ReefPole(table, this, false);
        }

        public ReefPole getRightPole() {
            return new ReefPole(table, this, true);
        }

        public double getAlgaeHeight() {
            return ((int) faceId - 'a') % 2 == 0 ? ElevatorConstants.kLowAlgaeHeight
                    : ElevatorConstants.kHighAlgaeHeight;
        }

        public boolean getAlgaeStatus() {
            return hasAlgae;
        }

        @Override
        public Pose2d getLocation() {
            return centerFace;
        }

        @Override
        public void updateState() {
            hasAlgae = algaeTopic.getEntry(false).get();
        }

        public static Optional<ReefFace> getNearest(Pose2d pose, boolean withAlgae) {
            Optional<ReefFace> nearest = Optional.empty();

            for (int i = 0; i < 6; i++) {
                var face = faces[i];

                if (withAlgae && !face.hasAlgae)
                    continue;

                if (nearest.isPresent()) {
                    var other = nearest.get();

                    if (getDistance(pose, other.getLocation()) < getDistance(pose, face.getLocation()))
                        nearest = Optional.of(face);
                } else
                    nearest = Optional.of(face);
            }

            return nearest;
        }
    }

    public static class CoralStation implements SelfDriveTarget {
        public static enum IntakePosition {
            Inner, Middle, Outer;
        }

        private final AllianceSide side;
        private final IntakePosition position;
        private final BooleanTopic hasCoralTopic;
        private boolean hasCoral;

        public CoralStation(AllianceSide allianceSide, IntakePosition stationPosition) {
            side = allianceSide;
            position = stationPosition;

            hasCoralTopic = State.stateTable.getSubTable("hpstations").getBooleanTopic(
                    (allianceSide == AllianceSide.Left ? "l" : "r") + "status");
        }

        @Override
        public Pose2d getLocation() {
            final Pose2d basePose = side == AllianceSide.Left ? FieldConstants.CoralStation.leftCenterFace
                    : FieldConstants.CoralStation.rightCenterFace;

            Translation2d shift = new Translation2d(0.4, -0.4);
            if (position != IntakePosition.Middle)
                shift = new Translation2d(shift.getX(), shift.getY() * (position == IntakePosition.Inner ? -1 : 1));
            else
                shift = new Translation2d();

            if (side == AllianceSide.Right)
                shift = new Translation2d(shift.getX() * -1, shift.getY());

            return AllianceFlipUtil.apply(basePose.plus(new Transform2d(shift, Rotation2d.kZero)));
        }

        @Override
        public void updateState() {
            hasCoral = hasCoralTopic.getEntry(false).get();
        }

        public boolean hasMoreCoral() {
            return hasCoral;
        }

        public AllianceSide getSide() {
            return side;
        }
    }

    public Pose2d getLocation();

    public void updateState();
}
