// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Vision;
import frc.robot.util.AllianceFlipUtil;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.events.EventTrigger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.net.WebServer;
import java.nio.file.Paths;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeRemover;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController driverController = new CommandXboxController(0);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final EndEffector endEffector = new EndEffector();
        public final Vision vision = new Vision();
        public final Elevator elevator = new Elevator();
        public final AlgaeRemover algaeRemover = new AlgaeRemover();
        public final Field2d field = new Field2d();

        private final Trigger coralTrigger = new Trigger(endEffector::coralInPosition);
        private final SendableChooser<Command> autoChooser = new SendableChooser<>();

        public RobotContainer() {
                // create event triggers for autos to use
                new EventTrigger("elevatorl4").whileTrue(elevator.setElevatorTarget(ElevatorConstants.kL4Height));
                new EventTrigger("elevatorstow").whileTrue(elevator.setElevatorTarget(ElevatorConstants.kStowedHeight));
                new EventTrigger("score").whileTrue(new WaitCommand(1.0).andThen(endEffector.ejectCoral()));
                new EventTrigger("score_safe").whileTrue(endEffector.scoreSafe(elevator::isAtHeight));

                algaeRemover.setDefaultCommand(algaeRemover.pivotAlgaeArm(AlgaeConstants.kAlgaeStowedRotation));
                endEffector.setDefaultCommand(endEffector.centerCoral());

                configureBindings();
                WebServer.start(
                                5801,
                                Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(), "hud")
                                                .toString());

                SmartDashboard.putData(field);

                PathfindingCommand.warmupCommand().schedule();

                autoChooser.setDefaultOption("Middle 1 Piece", new PathPlannerAuto("Mobility + AL"));
                // autoChooser.addOption("Cross Line", new MobilityAuto(drivetrain));
                autoChooser.addOption("Taxi", new PathPlannerAuto("Taxi"));
                autoChooser.addOption("Three Piece", new PathPlannerAuto("1"));
                autoChooser.addOption("Score from 1", new PathPlannerAuto("Score from 1"));
                autoChooser.addOption("Taxi + CL", new PathPlannerAuto("Taxi + CL"));
                SmartDashboard.putData("Auto Chooser", autoChooser);

                SmartDashboard.putString("Aligned X", "Unknown");
                SmartDashboard.putString("Aligned Y", "Unknown");
                DriverStation.silenceJoystickConnectionWarning(true);
        }

        public void periodic() {
                for (var pose : vision.getVisionResults()) {
                        drivetrain.addVisionMeasurement(pose.getPose(), pose.getTime(),
                                        pose.getVisionMeasurementStdDevs());
                }

                var xDiff = AllianceFlipUtil.apply(drivetrain.getState().Pose).getX() - 7.1;
                var yDiff = AllianceFlipUtil.apply(drivetrain.getState().Pose).getY() - 1.9;

                SmartDashboard.putString("Aligned X", xDiff < -0.01 ? "Out (to cages)"
                                : (xDiff > 0.01 ? "Closer (away from cages)" : "Aligned"));
                SmartDashboard.putString("Aligned Y", yDiff < -0.01 ? "Left" : (yDiff > 0.01 ? "Right" : "Aligned"));
                SmartDashboard.putString("Aligned X (num)", xDiff);
                SmartDashboard.putString("Aligned Y (num)", yDiff);

                field.setRobotPose(drivetrain.getState().Pose);
        }

        private void configureBindings() {
                coralTrigger.onTrue(Commands
                                .race(Commands.runOnce(() -> driverController.setRumble(RumbleType.kBothRumble,
                                                OperatorConstants.kRumbleStrength)), new WaitCommand(0.5))
                                .andThen(Commands.run(() -> driverController.setRumble(RumbleType.kBothRumble, 0))));

                driverController.rightTrigger().whileTrue(endEffector.ejectCoral()).onFalse(endEffector.centerCoral());
                driverController.leftTrigger()
                                .whileTrue(algaeRemover.pivotAlgaeArm(AlgaeConstants.kAlgaeExtendedRotation))
                                .whileFalse(algaeRemover.pivotAlgaeArm(AlgaeConstants.kAlgaeStowedRotation));
                // driverController.leftTrigger().whileFalse(algaeRemover.reset());

                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(
                                                                Math.copySign(Math.pow(driverController.getLeftY(), 2),
                                                                                -driverController.getLeftY())
                                                                                * MaxSpeed) // Drive forward with
                                                                                            // negative Y (forward)
                                                .withVelocityY(
                                                                Math.copySign(Math.pow(driverController.getLeftX(), 2),
                                                                                -driverController.getLeftX())
                                                                                * MaxSpeed) // Drive left with negative
                                                                                            // X (left)
                                                .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive
                                                                                                                    // counterclockwise
                                                                                                                    // with
                                                                                                                    // negative
                                                                                                                    // X
                                                                                                                    // (left)
                                ));

                driverController.povDown().whileTrue(drivetrain.applyRequest(() -> brake));
                driverController.povRight().whileTrue(drivetrain.applyRequest(() -> point
                                .withModuleDirection(new Rotation2d(-driverController.getLeftY(),
                                                -driverController.getLeftX()))));

                driverController.a().whileTrue(elevator.setElevatorTarget(ElevatorConstants.kL1Height))
                                .onFalse(elevator.setElevatorTarget(ElevatorConstants.kStowedHeight));
                driverController.x().whileTrue(elevator.setElevatorTarget(ElevatorConstants.kL2Height))
                                .onFalse(elevator.setElevatorTarget(ElevatorConstants.kStowedHeight));
                driverController.b().whileTrue(elevator.setElevatorTarget(ElevatorConstants.kL3Height))
                                .onFalse(elevator.setElevatorTarget(ElevatorConstants.kStowedHeight));
                driverController.y().whileTrue(elevator.setElevatorTarget(ElevatorConstants.kL4Height))
                                .onFalse(elevator.setElevatorTarget(ElevatorConstants.kStowedHeight));

                // TODO: these should only be enabled for testing/auto tuning.
                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                // driverController.back().and(driverController.y())
                // .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // driverController.back().and(driverController.x())
                // .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // driverController.start().and(driverController.y())
                // .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // driverController.start().and(driverController.x())
                // .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // reset the field-centric heading on start button press
                driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // Driver Right Bumper: Approach nearest right-side reef branch
                driverController.rightBumper().whileTrue(joystickApproach(
                                () -> FieldConstants.getNearestReefBranch(drivetrain.getState().Pose, ReefSide.RIGHT)));

                // Driver Left Bumper: approach nearest left-side reef branch
                driverController.leftBumper().whileTrue(joystickApproach(
                                () -> FieldConstants.getNearestReefBranch(drivetrain.getState().Pose, ReefSide.LEFT)));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public void ConfigureNamedCommands() {
                NamedCommands.registerCommand("score", endEffector.ejectCoral());
                NamedCommands.registerCommand("elevator stow",
                                elevator.setElevatorTarget(ElevatorConstants.kStowedHeight));
                NamedCommands.registerCommand("elevator L3", elevator.setElevatorTarget(ElevatorConstants.kL3Height));
                NamedCommands.registerCommand("elevatorl4", elevator.setElevatorTarget(ElevatorConstants.kL4Height));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void teleInit() {
                elevator.setElevatorTarget(ElevatorConstants.kStowedHeight);
        }

        private Command joystickApproach(Supplier<Pose2d> approachPose) {
                return DriveCommands.joystickApproach(drivetrain, () -> driverController.getLeftY(), approachPose);
        }
}
