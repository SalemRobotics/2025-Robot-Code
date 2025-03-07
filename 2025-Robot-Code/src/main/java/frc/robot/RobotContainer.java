// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.MobilityAuto;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Vision;
import frc.robot.util.AllianceFlipUtil;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.events.EventTrigger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.net.WebServer;
import java.nio.file.Paths;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeRemover;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
        /*
         * AJ reccomendations(NO JACK ATKINS):
         * - Add short delay (1/4 sec max, check if lower times work to eliminate
         * momentum) and add auto positioning of the coral to a "best" place (using
         * breakbeams and state in EndEffector)
         * - When coral is at optimal position or close, rumble the controller (should
         * be immediately unset as soon as neither beam sees the coral)
         * 
         * JACK ATKINS: WRITE AUTO-ALIGN CODE FOR THE REEF!!! THIS IS OF TOP
         * IMPORTANCE!!! The tasks above are for extras & other members to do, as
         * auto-align is essential. After that, we should tune(slow down) velocities
         * and after that you should fix the zeroing of the drivetrain
         */
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

        public RobotContainer() {
                new EventTrigger("elevatorl4").whileTrue(elevator.setElevatorTarget(ElevatorConstants.kL4Height));
                new EventTrigger("elevatorstow").whileTrue(elevator.setElevatorTarget(ElevatorConstants.kStowedHeight));
                new EventTrigger("score").whileTrue(endEffector.ejectCoral());
                endEffector.setDefaultCommand(endEffector.centerCoral());
                algaeRemover.setDefaultCommand(algaeRemover.pivotAlgaeArm(AlgaeConstants.kAlgaeStowedRotation));
                configureBindings();
                WebServer.start(
                                5801,
                                Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(), "hud")
                                                .toString());

                SmartDashboard.putData(field);

                PathfindingCommand.warmupCommand().schedule();
        }

        public void periodic() {
                ArrayList<VisionHelper> cam1Results = vision.getCurrentPositionCam1();
                ArrayList<VisionHelper> cam2Results = vision.getCurrentPositionCam2();

                for (int i = 0; i < cam1Results.size(); i++) {
                        VisionHelper cam1Result = cam1Results.get(i);
                        if (cam1Result.getPose().isPresent()) {
                                drivetrain.addVisionMeasurement(cam1Result.getPose().get(), cam1Result.getTime());
                        }
                }
                for (int i = 0; i < cam2Results.size(); i++) {
                        VisionHelper cam2Result = cam2Results.get(i);
                        if (cam2Result.getPose().isPresent()) {
                                drivetrain.addVisionMeasurement(cam2Result.getPose().get(), cam2Result.getTime());
                        }
                }

                Pose2d currentPose = drivetrain.getState().Pose;
                AllianceFlipUtil.apply(currentPose);
                for (int i = 0; i < 6; i++) {
                        SmartDashboard.putString("Blue Scoring Pose " + i, FieldConstants.blueCenterFaces[i].toString());
                        SmartDashboard.putString("Red Scoring Pose " + i, FieldConstants.redCenterFaces[i].toString());
                }

                int face;
                if (AllianceFlipUtil.shouldFlip()) {
                        var closestFace = currentPose.nearest(Arrays.asList(FieldConstants.redCenterFaces));
                        face = Arrays.asList(FieldConstants.redCenterFaces).indexOf(closestFace);
                } else {
                        Pose2d closestFace = currentPose.nearest(Arrays.asList(FieldConstants.blueCenterFaces));
                        face = Arrays.asList(FieldConstants.blueCenterFaces).indexOf(closestFace);
                }

                field.setRobotPose((AllianceFlipUtil.shouldFlip() 
                        ? FieldConstants.redCenterFaces 
                        : FieldConstants.blueCenterFaces)[face]);

                SmartDashboard.putString("Robot Pose", "X: " + drivetrain.getState().Pose.getX() + 
                        ", Y: " + drivetrain.getState().Pose.getY() + ", Heading: " +
                        drivetrain.getState().Pose.getRotation());
                
                Pose2d flippedPose = AllianceFlipUtil.apply(drivetrain.getState().Pose);
                SmartDashboard.putString("Flipped Pose", "X: " + flippedPose.getX() + 
                        ", Y: " + flippedPose.getY() + ", Heading: " +
                        flippedPose.getRotation());

                Pose2d leftPose = drivetrain.getScoringPose(false);
                SmartDashboard.putString("Left Score Pose", "X: " + leftPose.getX() + 
                        ", Y: " + leftPose.getY() + ", Heading: " +
                        leftPose.getRotation());
                Pose2d rightPose = drivetrain.getScoringPose(true);
                SmartDashboard.putString("Right Score Pose", "X: " + rightPose.getX() + 
                        ", Y: " + rightPose.getY() + ", Heading: " +
                        rightPose.getRotation());
        }

        private void configureBindings() {

                driverController.rightTrigger().whileTrue(endEffector.ejectCoral()).onFalse(endEffector.centerCoral());
                driverController.leftTrigger()
                                .whileTrue(algaeRemover.pivotAlgaeArm(AlgaeConstants.kAlgaeExtendedRotation))
                                .whileFalse(algaeRemover.pivotAlgaeArm(AlgaeConstants.kAlgaeStowedRotation));

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

                // driverController.a().onTrue(Commands.run(() -> endEffector.setEjectSpeed(EndEffectorConstants.kSlowEjectSpeed), endEffector))
                //                 .onFalse(Commands.run(() -> endEffector.setEjectSpeed(EndEffectorConstants.kDefaultEjectSpeed), endEffector));
                // driverController.y().onTrue(Commands.run(() -> endEffector.setEjectSpeed(EndEffectorConstants.kFastEjectSpeed), endEffector))
                //                 .onFalse(Commands.run(() -> endEffector.setEjectSpeed(EndEffectorConstants.kDefaultEjectSpeed), endEffector));

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
                //                 .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // driverController.back().and(driverController.x())
                //                 .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // driverController.start().and(driverController.y())
                //                 .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // driverController.start().and(driverController.x())
                //                 .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // reset the field-centric heading on start button press
                driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                driverController.leftBumper().onTrue(drivetrain.moveToScorePose(false));
                driverController.rightBumper().onTrue(drivetrain.moveToScorePose(true));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public void ConfigureNamedCommands() {
                NamedCommands.registerCommand("score", endEffector.ejectCoral());
                NamedCommands.registerCommand("elevator stow", elevator.setElevatorTarget(ElevatorConstants.kStowedHeight));
                NamedCommands.registerCommand("elevator L3", elevator.setElevatorTarget(ElevatorConstants.kL3Height));
                NamedCommands.registerCommand("elevatorl4", elevator.setElevatorTarget(ElevatorConstants.kL4Height));
        }

        public Command getAutonomousCommand() {
                return new PathPlannerAuto("Mobility + AL");
        }
}
