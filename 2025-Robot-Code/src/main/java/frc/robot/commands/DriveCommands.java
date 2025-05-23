package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;

public final class DriveCommands {
    public static class DriveCommandPID {
        public static record DrivePID(double p, double i, double d) {
        }

        DrivePID position = new DrivePID(2, 0, 1);
        PIDController positionController = new PIDController(position.p, position.i, position.d);
        DrivePID rotation = new DrivePID(2, 0, 1);
        PIDController rotationController = new PIDController(rotation.p, rotation.i, rotation.d);
        
        public DriveCommandPID() {
            rotationController.enableContinuousInput(Math.PI, Math.PI);
        }
        public DriveCommandPID withPositionConstants(double pP, double pI, double pD) {
            position = new DrivePID(pP, pI, pD);
            positionController = new PIDController(pP, pI, pD);
            return this;
        }    
        public DriveCommandPID withRotationConstants(double rP, double rI, double rD) {
            rotation = new DrivePID(rP, rI, rD);
            rotationController = new PIDController(rP, rI, rD);
            rotationController.enableContinuousInput(Math.PI, Math.PI);
            return this;
        }

        public PIDController getPositionController() {
            return positionController;
        }
        public PIDController getRotationController() {
            return rotationController;
        }
    }

    public static Command straightTowards(Drivetrain drive, Supplier<Pose2d> to, DriveCommandPID pid) {
        final var posePID = pid.getPositionController();
        final var rotPID = pid.getRotationController();

        return Commands.run(() -> {
            final Pose2d target = to.get();
            final Pose2d pose = drive.getPose();

            double x = posePID.calculate(pose.getX(), target.getX());
            double y = posePID.calculate(pose.getY(), target.getY());
            double rot = rotPID.calculate(pose.getRotation().getRadians(), target.getRotation().getRadians());

            final var req = new SwerveRequest.FieldCentric()
                .withVelocityX(x)
                .withVelocityY(y)
                .withRotationalRate(rot);

            drive.setControl(req);
        });
    }
}
