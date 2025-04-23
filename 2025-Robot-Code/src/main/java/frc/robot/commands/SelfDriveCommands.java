package frc.robot.commands;

import static frc.robot.util.Utilities.getDistance;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.selfdriving.Objective;
import frc.robot.subsystems.AlgaeRemover;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.util.Allocated;

public final class SelfDriveCommands {
    private static class ShiftingApproachTarget {
        private final Pose2d targetPose;
        private final Pose2d startFrom;
        private final double initialDistance;
        private Pose2d initalPose;
        /**
         * Whether or not the robot has ever been within 0.4m while this object has existed
         */
        private boolean hasMadeTarget = false;
        private boolean wasNearTarget = false;

        public ShiftingApproachTarget(Pose2d actual, Pose2d startShifting, Pose2d target) {
            initalPose = actual;
            targetPose = target;
            startFrom = startShifting;
            initialDistance = getDistance(initalPose, targetPose);
        }

        public Pose2d getTarget(Pose2d current) {
            if (!wasNearTarget) wasNearTarget = MathUtil.isNear(0, getDistance(current, startFrom), 0.4);
            if (hasMadeTarget && !wasNearTarget)
                return startFrom;

            double newDist = getDistance(current, targetPose);
            double progress = newDist / initialDistance;

            // we get the progression from startTarget -> endTarget, adding 0.05 to make
            // sure if the robot was exactly in the
            // right position it doesn't stop the robot
            double t = MathUtil.clamp(progress + 0.05, 0, 1);
            Translation2d interpolatedTranslation = startFrom.getTranslation().interpolate(targetPose.getTranslation(),
                    t);
            Rotation2d interpolatedRotation = startFrom.getRotation().interpolate(targetPose.getRotation(), t);

            var target = new Pose2d(interpolatedTranslation, interpolatedRotation);
            hasMadeTarget = true;
            return target;
        }
    }

    public static Command selfDrivingScore(
            Objective.Scoring objective,
            Drivetrain drive,
            EndEffector endEffector,
            AlgaeRemover algaeRemover,
            Elevator elevator) {
        Supplier<Pose2d> robotPose = () -> drive.getState().Pose;
        Pose2d scoringPose = objective.getScoringPose();
        ShiftingApproachTarget targetSupplier = new ShiftingApproachTarget(robotPose.get(), objective.targetQuadrant().middle(), scoringPose);

        if (endEffector.hasCoral())
            return selfDrivingScore(Objective.nearestCoralObjective(robotPose.get(), true), drive, endEffector,
                    algaeRemover, elevator);
        else if (algaeRemover.hasAlgae())
            return selfDrivingScore(Objective.nearestBargeObjective(robotPose.get()), drive, endEffector, algaeRemover,
                    elevator);

        final var velocityConstraints = new TrapezoidProfile.Constraints(8, 20);

        final ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, velocityConstraints);
        xController.setTolerance(0.08);
        final ProfiledPIDController yController = new ProfiledPIDController(2, 0, 0, velocityConstraints);
        yController.setTolerance(0.08);

        final ProfiledPIDController angleController = new ProfiledPIDController(2, 0, 0.1,
                new TrapezoidProfile.Constraints(8, 20));
        angleController.setTolerance(Units.degreesToRadians(0.5));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        final Allocated<Boolean> hasScored = new Allocated<>(false);
        final Allocated<Boolean> alreadyScheduledPreparation = new Allocated<>(false);

        return Commands.run(() -> {
            Pose2d robot = robotPose.get();
            Pose2d target = targetSupplier.getTarget(robot);

            double velocityX = xController.calculate(robot.getX(), target.getX());
            double velocityY = yController.calculate(robot.getY(), target.getY());
            double angle = angleController.calculate(robot.getRotation().getRadians(),
                    target.getRotation().getRadians());

            SwerveRequest request = new SwerveRequest.FieldCentric()
                    .withVelocityX(velocityX)
                    .withVelocityY(velocityY)
                    .withRotationalRate(angle)
                    .withDeadband(0.075);

            drive.setControl(request);

            var progress = getDistance(robot, target);
            if (objective.startPreparation(progress) && !alreadyScheduledPreparation.get()) {
                alreadyScheduledPreparation.set(true);
                objective.preparationCommand(endEffector, elevator, algaeRemover).schedule();
            }

            if (objective.hasCompleted())
                hasScored.set(true);
        }).until(() -> hasScored.get());
    }

    public static Command selfDrivingIntake(
            Objective.Intaking objective,
            Drivetrain drive,
            EndEffector endEffector,
            AlgaeRemover algaeRemover,
            Elevator elevator) {
        Allocated<Boolean> hasIntook = new Allocated<>(false);

        return Commands.run(() -> {
            SwerveRequest req = new SwerveRequest.RobotCentric()
                    .withVelocityX(0.01)
                    .withVelocityY(0.01);
            drive.setControl(req);
            hasIntook.set(true);
        }).until(() -> hasIntook.get());
    }
}
