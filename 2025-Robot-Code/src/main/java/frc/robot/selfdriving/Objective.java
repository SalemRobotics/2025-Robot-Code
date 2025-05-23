package frc.robot.selfdriving;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

        public boolean runScoreCommand(double progress);

        public Command scoringCommand(Allocated<Boolean> marker, EndEffector endEffector, AlgaeRemover remover,
                Elevator elevator);

        public Command finalizeCommand(EndEffector endEffector, AlgaeRemover remover, Elevator elevator);

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
                    .andThen(Commands.waitSeconds(0.05), endEffector.scoreSafe(elevator::isAtHeight))
                    .finallyDo(() -> marker.set(true));
        }

        @Override
        public Command finalizeCommand(EndEffector endEffector, AlgaeRemover remover, Elevator elevator) {
            return elevator.setElevatorTarget(ElevatorConstants.kStowedHeight);
        }

        @Override
        public boolean runScoreCommand(double progress) {
            return progress >= 0.6;
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
     * MAYBE: - SuperCycleObjective extends Scoring - grabs an algae from the reef then
     * scores on a reef pole
     * Equal to: [Drive up to reef, AlgaeObjective, CoralObjective]
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

        public static Optional<Intaking> nearestBargeIntake(Pose2d pose, AllianceSide sidePreference) {
            for (int face = sidePreference == AllianceSide.Left ? 0 : 3, times = 0; times < 6; face = ++face % 6, times++) {
                ReefFace reefFace = SelfDriveTarget.faces.get(face);
                if (reefFace.getAlgaeStatus())
                    return Optional.of(Intaking.create(reefFace));
            }

            return Optional.empty();
        }
    }
}
