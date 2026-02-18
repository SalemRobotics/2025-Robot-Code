package frc.robot.commands;

import static frc.robot.commands.DriveCommands.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public final class JoystickApproachCommand extends Command {
  private final Drive drive;
  private final DoubleSupplier ySupplier;
  private final Supplier<Pose2d> targetSupplier;

  private Pose2d targetPose2d;
  private Pose2d relativePose2d;
  private Rotation2d targetRotation2d;

  boolean running = false;

  static final double DEADBAND = 0.1;

  private final ProfiledPIDController angleController =
      new ProfiledPIDController(ANGLE_KP, 0, ANGLE_KD, ANGLE_CONSTRAINTS);

  private final ProfiledPIDController alignController =
      new ProfiledPIDController(DRIVE_KP, 0, DRIVE_KD, DRIVE_CONSTRAINTS);

  public JoystickApproachCommand(
      Drive drive, DoubleSupplier ySupplier, Supplier<Pose2d> targetSupplier) {
    this.drive = drive;
    this.ySupplier = ySupplier;
    this.targetSupplier = targetSupplier;

    alignController.setTolerance(POSITION_TOLERANCE);
    angleController.setTolerance(ANGLE_TOLERANCE);

    angleController.enableContinuousInput(-Math.PI, Math.PI);
    alignController.setGoal(0);

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    alignController.reset(0);
    angleController.reset(drive.getPose().getRotation().getRadians());
    targetPose2d = targetSupplier.get();

    Logger.recordOutput("AutoAlign/Approach/Target", targetPose2d);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    running = true;
    relativePose2d = drive.getPose().relativeTo(targetPose2d);
    targetRotation2d = targetPose2d.getRotation();

    // Calculate lateral linear velocity
    Translation2d offsetVector =
        new Translation2d(0, alignController.calculate(relativePose2d.getY()));

    // Calculate total linear velocity
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(-ySupplier.getAsDouble(), 0)
            .div(2)
            .times(drive.getMaxLinearSpeedMetersPerSec())
            .plus(offsetVector)
            .rotateBy(targetRotation2d);

    // Calculate angular speed
    double omega =
        angleController.calculate(
            drive.getRotation().getRadians(),
            targetRotation2d.rotateBy(Rotation2d.k180deg).getRadians());

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omega);

    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    running = false;
  }

  // Returns true when withing a lateral tolerance
  public boolean withinTolerance(double dist) {
    return running ? Math.abs(relativePose2d.getY()) < dist : false;
  }
}
