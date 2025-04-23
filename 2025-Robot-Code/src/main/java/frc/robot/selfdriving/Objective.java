package frc.robot.selfdriving;

import static frc.robot.util.Utilities.getDistance;

import java.util.function.BooleanSupplier;
import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.selfdriving.GameElementLocation.Quadrant;
import frc.robot.subsystems.AlgaeRemover;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public final class Objective {
    public static abstract class Scoring {
        public abstract Pose2d getScoringPose();
        public abstract Command preparationCommand(EndEffector endEffector, Elevator elevator, AlgaeRemover algaeRemover);
        public abstract boolean startPreparation(double progress);
        public abstract Quadrant targetQuadrant();
        public abstract boolean hasCompleted(); 
    }
    public static abstract class Intaking extends Object {
        public Intaking() {}
        public abstract Pose2d getIntakePose();
    }

    public static class CoralObjective extends Scoring {
        private final Pose2d reefPole;
        private boolean hasScored;

        public CoralObjective(Pose2d pole) {
            reefPole = pole;
        }

        @Override
        public Pose2d getScoringPose() {
            return reefPole;
        }

        @Override
        public Command preparationCommand(EndEffector endEffector, Elevator elevator, AlgaeRemover algaeRemover) {
            return elevator.setElevatorTarget(0).andThen(endEffector.scoreSafe(elevator::isAtHeight));
        }

        @Override
        public boolean startPreparation(double progress) {
            return progress >= 0.85;
        }
        
        @Override
        public Quadrant targetQuadrant() {
            return Quadrant.fromPoint(reefPole);
        }

        @Override
        public boolean hasCompleted() {
            return hasScored;
        }
    }

    public static Scoring nearestCoralObjective(Pose2d robotPose, boolean prioritizePoints) {
        throw new UnsupportedOperationException("Unimplemented method 'nearestCoralObjetive'");
    }

    public static Scoring nearestBargeObjective(Pose2d pose) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'nearestBargeObjective'");
    }
}
