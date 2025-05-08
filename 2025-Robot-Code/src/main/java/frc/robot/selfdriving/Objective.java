package frc.robot.selfdriving;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.FieldConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.selfdriving.SelfDriveTarget.AllianceSide;
import frc.robot.selfdriving.SelfDriveTarget.CoralStation;
import frc.robot.selfdriving.SelfDriveTarget.GamePiece;
import frc.robot.selfdriving.SelfDriveTarget.ReefFace;
import frc.robot.selfdriving.SelfDriveTarget.CoralStation.IntakePosition;
import frc.robot.subsystems.AlgaeRemover;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Allocated;

public interface Objective {

    public static interface Scoring extends Objective {
        public Pose2d getScoringPose();

        public Command scoringCommand(Allocated<Boolean> marker, EndEffector endEffector, AlgaeRemover remover,
                Elevator elevator);

        public Quadrant targetQuadrant();

        public boolean hasCompleted();
    }

    public static class CoralObjective implements Scoring {
        private final Pose2d reefPole;
        private boolean hasScored;
        private final double height;

        public CoralObjective(Pose2d pole, int level) {
            reefPole = pole;
            switch (level) {
                case 1:
                    height = ElevatorConstants.kL1Height;
                    break;
                case 2:
                    height = ElevatorConstants.kL2Height;
                    break;
                case 3:
                    height = ElevatorConstants.kL3Height;
                    break;
                case 4:
                    height = ElevatorConstants.kL4Height;
                    break;
                case 0:
                default:
                    height = 0;
                    break;
            }
        }

        @Override
        public Pose2d getScoringPose() {
            return reefPole;
        }

        @Override
        public Command scoringCommand(Allocated<Boolean> marker, EndEffector endEffector, AlgaeRemover remover,
                Elevator elevator) {
            return elevator.setElevatorTarget(height)
                    .andThen(endEffector.scoreSafe(elevator::isAtHeight));
        }

        @Override
        public Quadrant targetQuadrant() {
            return Quadrant.fromPose(reefPole);
        }

        @Override
        public boolean hasCompleted() {
            return hasScored;
        }
    }

    /*
     * TODO: Other objectives
     * - BargeObjective extends Scoring - barges an algae
     * - L1Objective extends Scoring - scores once on L1
     * - SuperCycleObjective extends Scoring - grabs an algae from the reef then
     * scores on a reef pole
     * Equal to: [Drive up to reef, AlgaeObjective, CoralObjective]
     * - L1IntakeObjective extends Intaking - gets a coral to score on L1 with
     * - IceCreamCoralObjective extends Intaking - grabs the coral from the best
     * determined icecream
     * - IceCreamAlgaeObjective extends Intaking - grabs an algae from the best
     * icecream
     * - CoralStationObjective extends Intaking - intakes a coral from the coral
     * station (waits until coral is first detected)
     * - AlgaeObjective extends Intaking - grabs an algae from the reef
     */

    public static class Intaking implements Objective {
        private final SelfDriveTarget objective;
        private final GamePiece piece;

        private Intaking(SelfDriveTarget target, GamePiece gamePiece) {
            objective = target;
            piece = gamePiece;
        }

        public static Intaking create(CoralStation station) {
            return new Intaking(station, GamePiece.Coral);
        }

        public static Intaking create(ReefFace face) {
            return new Intaking(face, GamePiece.Algae);
        }

        public Pose2d getFeedLocation() {
            return objective.getLocation();
        }

        public GamePiece getPieceType() {
            return piece;
        }

        public SelfDriveTarget getTarget() {
            return objective;
        }

        public Pose2d getStartingPose() {
            if (objective instanceof CoralStation station) {
                Pose2d pose = new Pose2d(3.5, 2.5, Rotation2d.kZero);
                if (station.getSide() == AllianceSide.Left)
                    pose = new Pose2d(pose.getX(), FieldConstants.fieldWidth - pose.getY(), pose.getRotation());
                return AllianceFlipUtil.apply(pose);
            } else if (objective instanceof ReefFace face)
                return Quadrant.fromPose(face.getLocation()).middle();
            else {
                System.err.println("Unknown Objective.Intaking target type: " + objective.getClass().getName());
                return AllianceFlipUtil.apply(new Pose2d(1d, FieldConstants.fieldWidth / 2, new Rotation2d()));
            }
        }

        public static Intaking nearestCoralStationIntake(Pose2d pose, IntakePosition position) {
            return Intaking.create(new CoralStation(
                    (pose.getY() < FieldConstants.fieldWidth / 2) ? AllianceSide.Right : AllianceSide.Left, position));
        }

        public static Intaking nearestBargeIntake(Pose2d pose, AllianceSide sidePreference) {
            throw new UnsupportedOperationException("Unimplemented method 'nearestBargeIntake'");
        }
    }
}
