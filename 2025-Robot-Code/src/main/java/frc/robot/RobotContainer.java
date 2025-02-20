// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MobilityAuto;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final EndEffector endEffector = new EndEffector();
    public final Vision vision = new Vision();
    public final Elevator elevator = new Elevator();

    private final AutoPicker mAutoPicker = new AutoPicker();
    

    public RobotContainer() {
        endEffector.setDefaultCommand(endEffector.centerCoral());
        configureBindings();
        mAutoPicker.initializeCommands("Basic Autos", new MobilityAuto(drivetrain));
    }

    public void periodic(){
        if(vision.getCurrentPositionCam1().getPose().isPresent()){
            drivetrain.addVisionMeasurement(vision.getCurrentPositionCam1().getPose().get(), vision.getCurrentPositionCam1().getTime());
        }
        if(vision.getCurrentPositionCam2().getPose().isPresent()){
            drivetrain.addVisionMeasurement(vision.getCurrentPositionCam2().getPose().get(), vision.getCurrentPositionCam2().getTime());
        }
            
    }

    private void configureBindings() {
        operatorController.b().whileTrue(endEffector.ejectCoral());
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverController.povDown().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.povRight().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        driverController.a().whileTrue(elevator.setElevatorTarget(ElevatorConstants.kL1Height)).onFalse(elevator.setElevatorTarget(ElevatorConstants.kStowedHeight));
        driverController.x().whileTrue(elevator.setElevatorTarget(ElevatorConstants.kL2Height)).onFalse(elevator.setElevatorTarget(ElevatorConstants.kStowedHeight));
        driverController.b().whileTrue(elevator.setElevatorTarget(ElevatorConstants.kL3Height)).onFalse(elevator.setElevatorTarget(ElevatorConstants.kStowedHeight));
        driverController.y().whileTrue(elevator.setElevatorTarget(ElevatorConstants.kL4Height)).onFalse(elevator.setElevatorTarget(ElevatorConstants.kStowedHeight));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return mAutoPicker.getSelected();
    }
}
