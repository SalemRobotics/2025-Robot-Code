package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;
import lombok.val;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;

/** Represents a single camera registered to VisionOdometry. */
sealed class RegisteredCamera permits RegisteredSimCamera {
  // camera data
  protected final PhotonCamera camera;
  protected final PhotonPoseEstimator poseEstimator;
  protected final DoubleFunction<Pose2d> robotPose;
  // a cached list
  final List<PoseObservation> poseObservations = new ArrayList<>();

  // camera data caches for the pose estimator
  private final Optional<Matrix<N3, N3>> cameraMatrix;
  private final Optional<Matrix<N8, N1>> distanceCoeffs;

  /**
   * Constructs a single registered camera. This method should only be called in {@link
   * VisionOdometry}, or by {@link RegisteredSimCamera}
   *
   * @param name The name of the camera
   * @param robotToCamera The transform vector to convert a camera-relative pose to a robot-relative
   *     pose.
   * @param robotPoseSupplier A function to get the robot's pose at a given timestamp.
   */
  protected RegisteredCamera(
      String name, Transform3d robotToCamera, DoubleFunction<Pose2d> robotPoseSupplier) {
    camera = new PhotonCamera(name);
    robotPose = robotPoseSupplier;

    poseEstimator =
        new PhotonPoseEstimator(APRILTAG_LAYOUT, MULTI_TAG_POSE_STRATEGY, robotToCamera);
    poseEstimator.setMultiTagFallbackStrategy(SINGLE_TAG_POSE_STRATEGY);
    poseEstimator.setTagModel(TargetModel.kAprilTag36h11);

    cameraMatrix = camera.getCameraMatrix();
    distanceCoeffs = camera.getDistCoeffs();
  }

  /**
   * Updates the camera's pose estimations, calculating poses for all unread results and adding them
   * to this camera's cache.
   */
  protected void update() {
    for (val result : camera.getAllUnreadResults()) {
      double timestamp = result.getTimestampSeconds();

      val pose = robotPose.apply(timestamp);
      poseEstimator.addHeadingData(timestamp, pose.getRotation());
      poseEstimator.setReferencePose(pose);

      val update = poseEstimator.update(result, cameraMatrix, distanceCoeffs);

      if (update.isPresent()) {
        val estimation = update.get();

        val targets = estimation.targetsUsed;
        val numTargets = targets.size();

        final int[] tagsUsed = new int[numTargets];
        double averageDistance = 0;
        double averageAmbiguity = 0;

        for (int i = 0; i < numTargets; i++) {
          val target = targets.get(i);

          tagsUsed[i] = target.fiducialId;
          averageDistance += target.bestCameraToTarget.getTranslation().getNorm();
          averageAmbiguity += target.poseAmbiguity;
        }

        val observation =
            new PoseObservation(
                estimation.estimatedPose,
                tagsUsed,
                averageDistance / numTargets,
                averageAmbiguity / numTargets,
                estimation.timestampSeconds);
        poseObservations.add(observation);
      }
    }
  }

  /**
   * Clears this camera's cache, assuming all results have been logged and processed by {@link
   * Vision#periodic()}
   */
  public void clearCache() {
    poseObservations.clear();
  }
}
