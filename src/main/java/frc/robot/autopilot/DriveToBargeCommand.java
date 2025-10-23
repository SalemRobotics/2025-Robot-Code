package frc.robot.autopilot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import lombok.val;
import org.littletonrobotics.junction.Logger;

@RequiredArgsConstructor
final class DriveToBargeCommand extends Command {
  private final Drive drive;
  private final DoubleSupplier ySupplier;
  private Rotation2d targetRotation;
  private double targetX;

  @Override
  public void initialize() {
    targetX = AllianceFlipUtil.applyX(FieldConstants.startingLineX - Units.inchesToMeters(13));
    targetRotation = AllianceFlipUtil.apply(Rotation2d.kZero);
  }

  @Override
  public void execute() {
    double yInput = MathUtil.applyDeadband(ySupplier.getAsDouble(), DriveCommands.DEADBAND);
    if (AllianceFlipUtil.shouldFlip()) {
      yInput = -yInput;
    }
    val vy = Math.copySign(yInput * yInput, yInput);

    val drivePose = drive.getPose();

    val vx = targetX - drivePose.getX();
    val omega = targetRotation.minus(drivePose.getRotation()).getRadians();

    Logger.recordOutput(
        "AutoAlign/DriveToBarge/Target", new Pose2d(drivePose.getY(), targetX, targetRotation));

    ChassisSpeeds speeds = new ChassisSpeeds(2 * vx, vy, omega);
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drivePose.getRotation());
    drive.runVelocity(speeds);
  }
}
