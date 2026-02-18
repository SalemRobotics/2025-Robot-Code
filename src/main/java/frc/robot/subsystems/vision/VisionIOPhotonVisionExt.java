package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleFunction;
import lombok.val;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;

public final class VisionIOPhotonVisionExt implements VisionIO {
  // Pose buffer of SwerveDrivePoseEstimator lasts for 1.5 seconds
  private static final double LATENCY_CUTOFF = 1.5;
  protected final PhotonCamera camera;
  private final Transform3d robotToCamera;
  private final PhotonPoseEstimator poseEstimator;
  private final DoubleFunction<Pose2d> robotPoseGetter;
  private final List<PoseObservation> results = new ArrayList<>(10);
  private final Set<Short> tagIds = new HashSet<>(APRILTAG_LAYOUT.getTags().size());

  /**
   * Creates a new photonvision camera I/O layer.
   *
   * @param idx The index of the camera's name and robot-to-camera
   * @param robotPose A function that gets the robot's pose at a given timestamp
   */
  public VisionIOPhotonVisionExt(int idx, DoubleFunction<Pose2d> robotPose) {
    camera = new PhotonCamera(CAMERA_NAMES[idx]);
    robotToCamera = ROBOT_TO_CAMERAS[idx];
    robotPoseGetter = robotPose;

    poseEstimator =
        new PhotonPoseEstimator(APRILTAG_LAYOUT, MULTI_TAG_POSE_STRATEGY, robotToCamera);
    poseEstimator.setMultiTagFallbackStrategy(SINGLE_TAG_POSE_STRATEGY);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    results.clear();
    tagIds.clear();

    val distCoeffs = camera.getDistCoeffs();
    val camMatrix = camera.getCameraMatrix();

    for (val update : camera.getAllUnreadResults()) {
      val timestamp = update.getTimestampSeconds();

      // Skip any vision updates with latency >= 1.5 seconds.
      if (Timer.getFPGATimestamp() - timestamp > LATENCY_CUTOFF) return;

      val robotPoseAtTime = robotPoseGetter.apply(timestamp);
      poseEstimator.setLastPose(robotPoseAtTime);
      poseEstimator.addHeadingData(timestamp, robotPoseAtTime.getRotation());

      val result = poseEstimator.update(update, camMatrix, distCoeffs, CONSTRAINED_SOLVEPNP_PARAMS);

      result.ifPresentOrElse(
          this::handlePoseEstimation,
          () -> {
            if (update.multitagResult.isPresent()) {
              handleMultitag(update.multitagResult.get(), update);
            } else if (update.targets.size() > 0) {
              handleSingleTag(update);
            }
          });
    }

    inputs.connected = camera.isConnected();
    inputs.tagIds = tagIds.stream().mapToInt(id -> (int) id).toArray();
    inputs.poseObservations = results.toArray(PoseObservation[]::new);
  }

  /**
   * Creates a {@link PoseObservation} from an EstimatedRobotPose and appends it to the list of
   * results.
   *
   * @param estimation The robot pose estimation to convert.
   */
  private void handlePoseEstimation(EstimatedRobotPose estimation) {
    val targets = estimation.targetsUsed;
    val numTargets = targets.size();
    val tagsUsed = new HashSet<>(numTargets);

    double ambiguity = 0;
    double tagDistance = 0;

    for (val target : targets) {
      val id = target.fiducialId;
      tagsUsed.add(id);

      // INVARIANT: Because the PhotonPoseEstimator only sees the tags we give it (so
      // APRILTAG_LAYOUT), we do not need to check if the tags used are present in the
      // layout.
      ambiguity += target.getPoseAmbiguity();
      tagDistance += target.bestCameraToTarget.getTranslation().getNorm();
    }

    ambiguity /= numTargets;
    tagDistance /= numTargets;

    val observation =
        new PoseObservation(
            estimation.timestampSeconds,
            estimation.estimatedPose,
            ambiguity,
            tagsUsed.size(),
            tagDistance,
            PoseObservationType.PHOTONVISION);

    results.add(observation);
  }

  private void handleMultitag(MultiTargetPNPResult multitag, PhotonPipelineResult result) {
    // Calculate the robot pose
    val fieldToCamera = multitag.estimatedPose.best;
    val fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
    val robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

    // Calculate the average tag distance
    double totalTagDistance = 0;
    for (val target : result.targets) {
      totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
    }

    // Add all of the tag IDs
    tagIds.addAll(multitag.fiducialIDsUsed);

    // Add the observation
    results.add(
        new PoseObservation(
            result.getTimestampSeconds(),
            robotPose,
            multitag.estimatedPose.ambiguity,
            multitag.fiducialIDsUsed.size(),
            totalTagDistance / result.targets.size(),
            PoseObservationType.PHOTONVISION));
  }

  private void handleSingleTag(PhotonPipelineResult singletag) {
    singletag.targets.sort((o1, o2) -> Double.compare(o1.poseAmbiguity, o2.poseAmbiguity));

    for (val target : singletag.targets) {
      val tagPose = APRILTAG_LAYOUT.getTagPose(target.fiducialId);

      if (tagPose.isPresent()) {
        // Get the tag pose out of the Optional
        val targetPose = tagPose.get();

        // Calculate the robot's pose from the camera-to-target and tag pose
        val cameraToTarget = target.bestCameraToTarget;
        val fieldToCamera = targetPose.plus(cameraToTarget.inverse());
        val robotPose = fieldToCamera.plus(robotToCamera.inverse());

        // Add the tag ID
        tagIds.add((short) target.fiducialId);

        results.add(
            new PoseObservation(
                singletag.getTimestampSeconds(),
                robotPose,
                target.poseAmbiguity,
                1,
                cameraToTarget.getTranslation().getNorm(),
                PoseObservationType.PHOTONVISION));

        // Stop evaluating targets
        return;
      }

      // Continue until we have a target whose pose does exist
      continue;
    }
  }
}
