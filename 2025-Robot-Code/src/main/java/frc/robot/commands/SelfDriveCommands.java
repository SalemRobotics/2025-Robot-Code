package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.util.Utilities.getDistance;

import java.util.Optional;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.DriveCommands.DriveCommandPID;
import frc.robot.generated.TunerConstants;
import frc.robot.selfdriving.Objective;
import frc.robot.selfdriving.Quadrant;
import frc.robot.selfdriving.SelfDriveTarget;
import frc.robot.selfdriving.SelfDriveTarget.AllianceSide;
import frc.robot.selfdriving.SelfDriveTarget.CoralStation;
import frc.robot.selfdriving.SelfDriveTarget.ReefFace;
import frc.robot.selfdriving.SelfDriveTarget.CoralStation.IntakePosition;
import frc.robot.selfdriving.SelfDriving;
import frc.robot.selfdriving.Objective.CoralObjective;
import frc.robot.selfdriving.Objective.Intaking;
import frc.robot.subsystems.AlgaeRemover;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.util.Allocated;

public final class SelfDriveCommands {
	// TODO: TUNE THESE
	private static final DriveCommandPID SELF_DRIVE_PID = new DriveCommandPID()
			.withPositionConstants(2, 0, 1)
			.withRotationConstants(2, 0, 1);
	private static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(.75).in(RadiansPerSecond);
	private static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

	private static final class ShiftingApproachTarget {
		private final Pose2d targetPose;
		private final Pose2d startFrom;
		private final double initialDistance;
		private boolean hasBeenNearStart = false;

		public ShiftingApproachTarget(Pose2d actual, Pose2d startShifting, Pose2d target) {
			/**
			 * The initial pose of the robot when the command started. This can be used
			 * to determine how much closer
			 */
			targetPose = target;
			if (getDistance(actual, target) < getDistance(startShifting, target)) {
				var proj = actual.relativeTo(target).getTranslation().times(0.1);
				startFrom = new Pose2d(proj, startShifting.getRotation());
			} else
				startFrom = startShifting;

			initialDistance = Math.min(getDistance(actual, targetPose), 1.5);
		}

		/**
		 * Determines whether or not the robot is near the starting point of this path
		 * 
		 * @param actual The actual pose of the robot
		 * @return
		 */
		public boolean isNearStartpoint(Pose2d actual) {
			return MathUtil.isNear(0, getDistance(actual, startFrom), 1);
		}

		public Pose2d getTarget(Pose2d current) {
			double newDistance = getDistance(current, targetPose);

			// ensure that we are close enough to the starting point before starting the
			// curved approach
			if (!isNearStartpoint(current) && !hasBeenNearStart)
				return startFrom;
			else
				hasBeenNearStart = true;

			// this(newDist / initialDistance) should NEVER be greater than 1
			double progress = 1 - MathUtil.clamp(newDistance / initialDistance, 0, 1);

			// we get the progression from startTarget -> endTarget, adding 0.05 to make
			// sure if the robot was exactly in the right position it doesn't stop the robot
			double clamped = MathUtil.clamp(progress + 0.05, 0, 1);
			Translation2d interpolatedTranslation = startFrom.getTranslation().interpolate(
					targetPose.getTranslation(),
					clamped);
			Rotation2d interpolatedRotation = startFrom.getRotation().interpolate(targetPose.getRotation(),
					clamped);

			var target = new Pose2d(interpolatedTranslation, interpolatedRotation);

			// TODO: tune this constant to get the greatest and most ACCURATE alignment to
			// the target while allowing for the greatest distance
			if (progress < 0.5) {
				// shift the target to the pose if we aren't halfway there (makes kind of an
				// J-shape path)
				// TODO: tune the amount we shift the pose by so that the PID is very agressive
				var shift = new Transform2d(target, current).times(0.1).times(1 - (progress / 0.5));

				target = target.plus(shift);
			}

			return target;
		}
	}

	private static Command reefApproach(
			Pose2d scoringPose,
			Drivetrain drive,
			boolean preferTopInRepositioning,
			Pose2d startFrom,
			DoubleSupplier driverX,
			DoubleSupplier driverY,
			DoubleSupplier driverRot,
			DoubleConsumer updateProgress) {
		final Pose2d startingPose = drive.getPose();

		final ShiftingApproachTarget targetSupplier = new ShiftingApproachTarget(startingPose, startFrom,
				scoringPose);
		final double initialDist = getDistance(startingPose, scoringPose);

		final TrapezoidProfile.Constraints pidConstraints = new TrapezoidProfile.Constraints(8, 20);

		final ProfiledPIDController poseController = new ProfiledPIDController(3, 0, 0, pidConstraints);
		poseController.setTolerance(0.75);

		final ProfiledPIDController angleController = new ProfiledPIDController(2, 0, 0.1, pidConstraints);
		angleController.setTolerance(Units.degreesToRadians(5));
		angleController.enableContinuousInput(-Math.PI, Math.PI);

		Command cmd = Commands.run(() -> {
			Pose2d robot = drive.getPose();
			Pose2d target = targetSupplier.getTarget(robot);
			double progress = 1 - MathUtil.clamp(getDistance(robot, target) / initialDist, 0, 1);

			updateProgress.accept(progress);

			double velocityX = poseController.calculate(robot.getX(), scoringPose.getX());
			double velocityY = poseController.calculate(robot.getY(), scoringPose.getY());
			double rotation = angleController.calculate(robot.getRotation().getRadians(), target.getRotation().getRadians());

			final double scale = Math.hypot(velocityX, velocityY);
			Translation2d direction = target.getTranslation().minus(robot.getTranslation());

			if (progress < 0.8) {
				Translation2d manual = new Translation2d(
						driverX.getAsDouble() * MAX_SPEED,
						driverY.getAsDouble() * MAX_SPEED)
						.times(0.5);

				direction = direction.plus(manual);

				
				rotation = rotation + (driverRot.getAsDouble() * MAX_ANGULAR_RATE);
			}

			SwerveRequest request = new SwerveRequest.FieldCentric()
					.withVelocityX(direction.getX() * scale)
					.withVelocityY(direction.getY() * scale)
					.withRotationalRate(rotation)
					.withDeadband(0.075);

			drive.setControl(request);
		}, drive);

		if (initialDist >= 1.5)
			return Quadrant.fromPose(startingPose)
					.driveToOther(Quadrant.fromPose(scoringPose), preferTopInRepositioning)
					.until(() -> getDistance(drive.getPose(), scoringPose) < 1.5).andThen(cmd);
		else
			return cmd;
	}

	public static Command coralStationIntake(
			CoralStation station,
			Drivetrain drive,
			EndEffector endEffector,
			DoubleSupplier driverX,
			DoubleSupplier driverY,
			DoubleSupplier driverRot) {
		Pose2d intakePose = station.getLocation();

		return Commands.race(
				DriveCommands.straightTowards(drive, () -> intakePose, SELF_DRIVE_PID),
				endEffector.shortCircuitingIntake(false));
	}

	/**
	 * Executes on a {@link frc.robot.selfdriving.Objective.Scoring Scoring
	 * Objective} and scores a single game piece
	 * 
	 * @param objective                The objective to accomplish
	 * @param drive                    The robot's drivetrain
	 * @param endEffector              The robot's end effector
	 * @param algaeRemover             The robot's algae remover arm
	 * @param elevator                 The robot's elevator
	 * @param selfDriver               The robot's self driving subsystem
	 * @param preferTopInRepositioning Whether or not to travel behind or in front
	 *                                 of the reef, from the driver's perspective
	 * @param startTargetFrom          Where to start a coral score or algae intake
	 *                                 projected target from
	 * @param preferredIntakePosition
	 * @param preferredAllianceSide    The preferred alliance side to intake an
	 *                                 Algae from
	 * @param driverX					Driver override in the X direction (away from drivestation)
	 * @param driverY					Driver override in the Y direction (size to side)
	 * @param driverRot					Driver override for rotation
	 * @return
	 */
	public static Command selfDrivingScore(
			Objective.Scoring objective,
			Drivetrain drive,
			EndEffector endEffector,
			AlgaeRemover algaeRemover,
			Elevator elevator,
			boolean preferTopInRepositioning,
			Pose2d startTargetFrom,
			IntakePosition preferredIntakePosition,
			AllianceSide preferredAllianceSide,
			DoubleSupplier driverX,
			DoubleSupplier driverY,
			DoubleSupplier driverRot) {
		Optional<Command> intakeBeforeRunning = Optional.empty();

		Allocated<Double> progress = new Allocated<>(0d);

		final Command driveCmd;
		if (objective instanceof CoralObjective) {
			if (!endEffector.hasCoral())
				intakeBeforeRunning = Optional.of(selfDrivingIntake(
						Intaking.nearestCoralStationIntake(drive.getPose(), preferredIntakePosition), drive,
						endEffector, algaeRemover, elevator, preferTopInRepositioning, driverX, driverY, driverRot));
			driveCmd = reefApproach(
					objective.getScoringPose(),
					drive,
					preferTopInRepositioning,
					startTargetFrom,
					driverX, driverY,
					driverRot,
					progress::set);
		} else {
			if (!endEffector.hasAlgae()) {
				var intaking = Intaking.nearestBargeIntake(drive.getPose(), preferredAllianceSide);
				if (intaking.isEmpty())
					return Commands.runOnce(() -> SelfDriveTarget.State.markAllAlgaeScored());
				intakeBeforeRunning = Optional.of(selfDrivingIntake(intaking.get(), drive, endEffector, algaeRemover,
						elevator, preferTopInRepositioning, driverX, driverY, driverRot));
			}
			Allocated<Boolean> startedScore = new Allocated<>(false);

			driveCmd = Quadrant.fromPose(drive.getPose())
					.driveToOther(Quadrant.Q4, preferTopInRepositioning)
					.until(() -> getDistance(drive.getPose(),
							Quadrant.Q4.middle()) <= (preferTopInRepositioning ? 0.25 : 0.75))
					.andThen(Commands.run(() -> {
						final var drivePID = new DriveCommands.DriveCommandPID();
						final Pose2d currentPose = drive.getPose();

						double x = drivePID.getPositionController().calculate(currentPose.getX(),
								FieldConstants.startingLineX);
						double y = driverY.getAsDouble();
						double rot = drivePID.getRotationController().calculate(currentPose.getRotation().getRadians(),
								Rotation2d.kZero.getRadians());

						final var req = new SwerveRequest.FieldCentric()
								.withVelocityX(x)
								.withVelocityY(y)
								.withRotationalRate(rot);

						drive.setControl(req);
					}, drive).until(() -> MathUtil.isNear(FieldConstants.startingLineX, drive.getPose().getX(), 0.05)),
							elevator.setElevatorTarget(ElevatorConstants.kL4Height)
									.alongWith(Commands.runOnce(() -> startedScore.set(true)),
											Commands.waitSeconds(0.7).andThen(
													endEffector.scoreBarge().alongWith(algaeRemover.stowArm()))))
					.alongWith(algaeRemover.deployArm().until(startedScore::get));
		}

		Allocated<Boolean> hasScored = new Allocated<>(false);

		return intakeBeforeRunning.orElseGet(Commands::none)
				.andThen(driveCmd.alongWith(Commands.waitUntil(() -> objective.runScoreCommand(progress.get()))
						.andThen(objective.scoringCommand(hasScored, endEffector, algaeRemover, elevator))))
				.andThen(objective.finalizeCommand(drive, endEffector, algaeRemover, elevator));
	}

	private static Command algaeIntakeCommand(SelfDriveTarget target, EndEffector effector, AlgaeRemover remover,
			Elevator elevator) {
		if (target instanceof ReefFace face) {
			return elevator.setElevatorTarget(face.getAlgaeHeight())
					.alongWith(remover.deployArm(), effector.algaeIntake()).until(effector::hasAlgae);
		} else
			return Commands.print("Invalid target type: " + target.getClass().getName());
	}

	public static Command selfDrivingIntake(
			Objective.Intaking objective,
			Drivetrain drive,
			EndEffector endEffector,
			AlgaeRemover algaeRemover,
			Elevator elevator,
			boolean preferTopInRepositioning,
			DoubleSupplier driverX,
			DoubleSupplier driverY,
			DoubleSupplier driverRot) {
		Allocated<Double> progress = new Allocated<>(0d);

		var target = objective.getTarget();

		if (target instanceof CoralStation station) {
			final Quadrant targetQuad = Quadrant.fromPose(objective.getFeedLocation());
			final Command reposition = Quadrant.fromPose(drive.getPose()).driveToOther(
					Quadrant.fromPose(objective.getFeedLocation()), preferTopInRepositioning);
			return reposition
					.until(() -> getDistance(drive.getPose(), targetQuad.middle()) <= 1)
					.andThen(coralStationIntake(station, drive, endEffector, driverX, driverY,
							driverRot));
		} else {
			// we know as an invariant that this has to be an algae target
			return reefApproach(
					objective.getFeedLocation(),
					drive,
					preferTopInRepositioning,
					objective.getStartingPose(),
					driverX, driverY,
					driverRot,
					progress::set)
					.alongWith(Commands.waitUntil(() -> progress.get() >= 0.7).andThen(
							algaeIntakeCommand(objective.getTarget(), endEffector, algaeRemover, elevator)));
		}
	}
}
