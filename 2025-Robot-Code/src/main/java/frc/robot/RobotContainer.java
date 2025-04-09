// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static edu.wpi.first.units.Units.*;

import frc.robot.FieldConstants.ReefSide;
import frc.robot.commands.DriveCommands;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.AlgaeRemover;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.net.WebServer;

import java.util.function.Supplier;
import java.nio.file.Paths;
import java.lang.Math;

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
        private final CommandXboxController operatorController = new CommandXboxController(1);

        private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(driverController);
        private final EndEffector endEffector = new EndEffector(driverController);
        private final Vision vision = new Vision();
        private final Elevator elevator = new Elevator();
        private final AlgaeRemover algaeRemover = new AlgaeRemover();
        private final Climber climber = new Climber();

        private final Field2d field = new Field2d();

        private final SendableChooser<PathPlannerAuto> autoChooser = new SendableChooser<>();

        private boolean mBargeMode = false;
        private final Trigger isBargeMode = new Trigger(() -> mBargeMode);

        public RobotContainer() {
                configureNamedCommands();

                configureBindings();
                WebServer.start(5801,
                                Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(), "hud")
                                                .toString());

                SmartDashboard.putData(field);

                PathfindingCommand.warmupCommand().schedule();

                autoChooser.setDefaultOption("Own Cage 3.5pc", new PathPlannerAuto("Own Cage 3.5pc"));
                autoChooser.addOption("Middle 1pc + 3 Barge", new PathPlannerAuto("Middle 1pc + 3 Barge"));
                autoChooser.addOption("Opps Cage 3.5pc", new PathPlannerAuto("Opps Cage 3.5pc"));
                autoChooser.addOption("Tush Push", new PathPlannerAuto("Tush Push"));

                SmartDashboard.putData("Auto Chooser", autoChooser);
                SmartDashboard.putString("Aligned X", "Unknown (in initialization)");
                SmartDashboard.putString("Aligned Y", "Unknown (in initialization)");
                SmartDashboard.putBoolean("L1 Direction", true);
                SmartDashboard.putData("Align Auto", drivetrain.AlignAuto(autoChooser));

                DriverStation.silenceJoystickConnectionWarning(true);
        }

        public void periodic() {
                for (var pose : vision.getVisionResults()) {
                        drivetrain.addVisionMeasurement(pose.getPose(), pose.getTime(),
                                        pose.getVisionMeasurementStdDevs());
                }

                field.setRobotPose(drivetrain.getState().Pose);
                
                SmartDashboard.putBoolean("In Algae Mode", mBargeMode);
                SmartDashboard.putNumber("Timer", DriverStation.getMatchTime());
        }

        private void configureBindings() {
                driverController.rightTrigger().and(isBargeMode.negate())
                                .whileTrue(endEffector.scoreCoral(driverController.y()));
                driverController.rightTrigger().and(isBargeMode)
                                .whileTrue(endEffector.scoreBarge().alongWith(algaeRemover.stowArm()));

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

                operatorController.a().whileTrue(climber.climb().alongWith(algaeRemover.dropArm())).onFalse(climber.stopMotor());
                operatorController.y().whileTrue(climber.declimb()).onFalse(climber.stopMotor());

                driverController.a().and(isBargeMode.negate())
                                .whileTrue(elevator.setElevatorTarget(ElevatorConstants.kL1Height)
                                .alongWith(Commands.waitSeconds(0.3)
                                .andThen(endEffector.scoreL1()).alongWith(Commands.waitSeconds(0.1).andThen(L1MoveCommand()))))
                                .onFalse(elevator.setElevatorTarget(ElevatorConstants.kStowedHeight));
                driverController.x().and(isBargeMode.negate())
                                .whileTrue(elevator.setElevatorTarget(ElevatorConstants.kL2Height))
                                .onFalse(elevator.setElevatorTarget(ElevatorConstants.kStowedHeight));
                driverController.b().and(isBargeMode.negate())
                                .whileTrue(elevator.setElevatorTarget(ElevatorConstants.kL3Height))
                                .onFalse(elevator.setElevatorTarget(ElevatorConstants.kStowedHeight));
                driverController.y().and(isBargeMode.negate())
                                .whileTrue(elevator.setElevatorTarget(ElevatorConstants.kL4Height))
                                .onFalse(elevator.setElevatorTarget(ElevatorConstants.kStowedHeight));

                driverController.x().and(isBargeMode)
                                .whileTrue(elevator.setElevatorTarget(ElevatorConstants.kLowAlgaeHeight))
                                .onFalse(elevator.setElevatorTarget(ElevatorConstants.kStowedHeight));
                driverController.b().and(isBargeMode)
                                .whileTrue(elevator.setElevatorTarget(ElevatorConstants.kHighAlgaeHeight))
                                .onFalse(elevator.setElevatorTarget(ElevatorConstants.kStowedHeight));
                driverController.y().and(isBargeMode)
                                .whileTrue(elevator.setElevatorTarget(ElevatorConstants.kL4Height)
                                                .alongWith(Commands.waitSeconds(0.7)
                                                                .andThen(endEffector.scoreBarge()
                                                                                .alongWith(algaeRemover.stowArm()))))
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
                driverController.rightBumper().and(isBargeMode.negate()).whileTrue(joystickApproach(
                                () -> FieldConstants.getNearestReefBranch(drivetrain.getState().Pose, ReefSide.RIGHT)));

                // Driver Left Bumper: approach nearest left-side reef branch
                driverController.leftBumper().and(isBargeMode.negate()).whileTrue(joystickApproach(
                                () -> FieldConstants.getNearestReefBranch(drivetrain.getState().Pose, ReefSide.LEFT)));

                // Driver Left Bumper and Barge Mode: approach Algae on current reef face
                driverController.leftBumper().and(isBargeMode).whileTrue(joystickApproach(
                                () -> FieldConstants.getNearestReefFace(drivetrain.getState().Pose)));
                driverController.rightBumper().and(isBargeMode).whileTrue(joystickApproach(
                                () -> FieldConstants.getNearestReefFace(drivetrain.getState().Pose)));

                driverController.leftTrigger().onTrue(
                                algaeRemover.deployArm().alongWith(Commands.runOnce(() -> mBargeMode = true),
                                                endEffector.algaeIntake()))
                                .onFalse(Commands.runOnce(() -> mBargeMode = false));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        private void configureNamedCommands() {
                // create named commands for autos to use
                NamedCommands.registerCommand("elevatorl4", elevator.setElevatorTarget(ElevatorConstants.kL4Height));

                NamedCommands.registerCommand("startalgae", algaeRemover.deployArm()
                                .alongWith(Commands.runOnce(() -> mBargeMode = true), endEffector.algaeIntake()));

                NamedCommands.registerCommand("endalgae", algaeRemover.stowArm(() -> false)
                                .alongWith(Commands.runOnce(() -> mBargeMode = false)));
                NamedCommands.registerCommand("algael2", elevator.setElevatorTarget(ElevatorConstants.kLowAlgaeHeight));
                NamedCommands.registerCommand("algael3",
                                elevator.setElevatorTarget(ElevatorConstants.kHighAlgaeHeight));
                NamedCommands.registerCommand("scorealgae", elevator.setElevatorTarget(ElevatorConstants.kL4Height)
                                .alongWith(Commands.waitSeconds(0.7).andThen(endEffector.scoreBarge()
                                                .alongWith(algaeRemover.stowArm()))));
                NamedCommands.registerCommand("elevatorstow",
                                elevator.setElevatorTarget(ElevatorConstants.kStowedHeight));
                NamedCommands.registerCommand("score", endEffector.autoScoreCoral());
                NamedCommands.registerCommand("score_safe", endEffector.scoreSafe(elevator::isAtHeight));
                NamedCommands.registerCommand("intake", endEffector.autoIntake());
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void autoInit() {
                endEffector.checkIfContainsCoral();
        }

        public void teleInit() {
                // ensure that barge mode doesn't carry through from a previous enable
                mBargeMode = false;
                // send the elevator to the stow position so AJ doesn't lose time manually
                // sending it down
                elevator.setElevatorTarget(ElevatorConstants.kStowedHeight).schedule();
                // set default commands here to prevent issues in auto, as in auto we want to
                // manually schedule all commands & states
                endEffector.setDefaultCommand(endEffector.teleIntake());
                algaeRemover.setDefaultCommand(algaeRemover.stowArm(driverController.leftTrigger()));
        }

        public void teleExit() {
                // remove default commands after teleop to prevent accidental cancellations by
                // default commands
                endEffector.removeDefaultCommand();
                algaeRemover.removeDefaultCommand();
        }

        /**
         * Makes sure that when entering any mode other than disabled, we don't jog the
         * coral out of the end effector
         */
        public void disabledExit() {
                endEffector.checkIfContainsCoral();
        }

        /**
         * Code to run while the robot is disabled. This code is for alignment purposes
         * and some other currently unused functionality.
         * 
         * TODO: make field alignment states actually useful
         */
        public void disabledPeriodic() {
                var xDiff = AllianceFlipUtil.apply(drivetrain.getState().Pose).getX() - 7.1;
                var yDiff = AllianceFlipUtil.apply(drivetrain.getState().Pose).getY() - 1.9;

                SmartDashboard.putString("Aligned X", xDiff < -0.01 ? "Out (to cages)"
                                : (xDiff > 0.01 ? "Closer (away from cages)" : "Aligned"));
                SmartDashboard.putString("Aligned Y", yDiff < -0.01 ? "Left" : (yDiff > 0.01 ? "Right" : "Aligned"));
                SmartDashboard.putNumber("Aligned X (num)", xDiff);
                SmartDashboard.putNumber("Aligned Y (num)", yDiff);
        }

        private Command joystickApproach(Supplier<Pose2d> approachPose) {
                return DriveCommands.joystickApproach(drivetrain, () -> driverController.getLeftY(), approachPose);
        }

        private Command L1MoveCommand() {
                return drivetrain.L1Move(() -> SmartDashboard.getBoolean("L1 Direction", false));
        }
}
