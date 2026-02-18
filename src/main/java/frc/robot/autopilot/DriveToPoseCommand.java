package frc.robot.autopilot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.val;
import org.littletonrobotics.junction.Logger;

public class DriveToPoseCommand extends Command {
  private final Drive drive;
  private final Supplier<Pose2d> poseSupplier;
  private final String logKey;

  @Getter private Pose2d targetPose;
  private double initialDistance;
  private Translation2d targetTranslation;
  private Rotation2d targetRotation;
  private boolean running = false;

  public DriveToPoseCommand(String name, Drive drive, Supplier<Pose2d> poseSupplier) {
    setName(name);
    logKey = "AutoAlign/" + name;

    this.drive = drive;
    this.poseSupplier = poseSupplier;

    targetPose = poseSupplier.get();
    targetTranslation = targetPose.getTranslation();
    targetRotation = targetPose.getRotation();

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    targetPose = poseSupplier.get();
    targetTranslation = targetPose.getTranslation();
    targetRotation = targetPose.getRotation();
    initialDistance = drive.getPose().getTranslation().getDistance(targetTranslation);
    running = true;

    Logger.recordOutput(logKey + "/Target", targetPose);
  }

  @Override
  public void execute() {
    val drivePose = drive.getPose();

    Logger.recordOutput(
        logKey + "/DistanceToTarget", drivePose.getTranslation().getDistance(targetTranslation));

    val diffTranslation = targetTranslation.minus(drivePose.getTranslation());

    Logger.recordOutput(logKey + "/vX", diffTranslation.getX());
    Logger.recordOutput(logKey + "/vY", diffTranslation.getY());

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            diffTranslation.getX(),
            diffTranslation.getY(),
            targetRotation.minus(drivePose.getRotation()).getRadians());

    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation());

    drive.runVelocity(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
  }

  public boolean isRunning() {
    return running;
  }

  public boolean withinTolerance(double tolerance) {
    return drive.getPose().getTranslation().getDistance(targetTranslation) <= tolerance;
  }

  public boolean madeProgress(double progress) {
    double newDist = drive.getPose().getTranslation().getDistance(targetTranslation);

    return (initialDistance - newDist) / initialDistance >= progress;
  }
}
