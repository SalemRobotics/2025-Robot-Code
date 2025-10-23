package frc.robot.autopilot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.end_effector.EndEffector;
import java.util.function.DoubleSupplier;
import lombok.val;
import org.littletonrobotics.junction.Logger;

public final class AutopilotCommands {
  private static final Translation2d LEFT_INSIDE_TRANSLATION = new Translation2d(-0.5, -0.375);
  private static final Translation2d LEFT_OUTSIDE_TRANSLATION = new Translation2d(0.5, 0.375);
  private static final Translation2d RIGHT_INSIDE_TRANSLATION = new Translation2d(-0.5, 0.375);
  private static final Translation2d RIGHT_OUTSIDE_TRANSLATION = new Translation2d(0.5, -0.375);

  public static Command driveToCoralStation(Drive drive, boolean left) {
    return new DriveToPoseCommand(
        "DriveToCS",
        drive,
        () -> {
          val drivePose = drive.getPose();
          val leftSide = drivePose.getY() > FieldConstants.fieldCenter.getY();
          // On the left side, "to the left" of the driver station is to the outside
          val inside = leftSide ? !left : left;

          Pose2d coralStation = FieldConstants.getNearestCoralStation(drive.getPose());
          Logger.recordOutput("AutoAlign/DriveToCS/StationPose", coralStation);
          Logger.recordOutput("AutoAlign/DriveToCS/ToInside", inside);
          Logger.recordOutput("AutoAlign/DriveToCS/OnAllianceLeft", leftSide);

          val stationTranslation = coralStation.getTranslation();

          Translation2d targetTranslation;
          if (leftSide) {
            targetTranslation =
                stationTranslation.plus(
                    inside ? LEFT_INSIDE_TRANSLATION : LEFT_OUTSIDE_TRANSLATION);
          } else {
            targetTranslation =
                stationTranslation.plus(
                    inside ? RIGHT_INSIDE_TRANSLATION : RIGHT_OUTSIDE_TRANSLATION);
          }

          return new Pose2d(targetTranslation, coralStation.getRotation());
        });
  }

  public static Command intakeCoral(
      boolean inside, Drive drive, Elevator elevator, EndEffector endEffector) {
    return Commands.parallel(
        driveToCoralStation(drive, inside), elevator.stow(), endEffector.teleIntake());
  }

  public static Command driveToBarge(Drive drive, DoubleSupplier ySupplier) {
    return new DriveToBargeCommand(drive, ySupplier);
  }
}
