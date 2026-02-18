package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.Statics;
import java.nio.file.Paths;
import java.util.Optional;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public final class VisionConstants {
  public static final double CAMERA_FPS = 30;

  public static final AprilTagFieldLayout APRILTAG_LAYOUT =
      Statics.initOrDefault(
          () ->
              new AprilTagFieldLayout(
                  Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "reeftags.json")),
          () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));

  public static final PoseStrategy MULTI_TAG_POSE_STRATEGY =
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
  public static final PoseStrategy SINGLE_TAG_POSE_STRATEGY = PoseStrategy.PNP_DISTANCE_TRIG_SOLVE;
  public static final Optional<ConstrainedSolvepnpParams> CONSTRAINED_SOLVEPNP_PARAMS =
      Optional.of(new ConstrainedSolvepnpParams(false, 10));

  public static final String[] CAMERA_NAMES = new String[] {"Left", "Right"};
  public static final Transform3d[] ROBOT_TO_CAMERAS =
      new Transform3d[] {
        new Transform3d(
            Units.inchesToMeters(11.58),
            Units.inchesToMeters(11.189),
            Units.inchesToMeters(8.25),
            new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-20))),
        new Transform3d(
            Units.inchesToMeters(11.58),
            Units.inchesToMeters(-11.189),
            Units.inchesToMeters(8.25),
            new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(15)))
      };

  public static final double maxAmbiguity = 0.3;
  public static final double maxZError = 0.5;
  public static final double linearStdDevBaseline = Units.inchesToMeters(5);
  public static final double angularStdDevBaseline = Units.degreesToRadians(10);

  public static final double[] cameraStdDevFactors = new double[] {1.2, 1.2};
}
