package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.APRILTAG_LAYOUT;
import static frc.robot.subsystems.vision.VisionConstants.CAMERA_NAMES;
import static frc.robot.subsystems.vision.VisionConstants.angularStdDevBaseline;
import static frc.robot.subsystems.vision.VisionConstants.cameraStdDevFactors;
import static frc.robot.subsystems.vision.VisionConstants.linearStdDevBaseline;
import static frc.robot.subsystems.vision.VisionConstants.maxAmbiguity;
import static frc.robot.subsystems.vision.VisionConstants.maxZError;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.concurrent.locks.Lock;
import lombok.val;
import org.littletonrobotics.junction.Logger;

public final class Vision extends SubsystemBase {
  static boolean visionEnabled = true;

  public static final void enable() {
    visionEnabled = true;
  }

  public static final void disable() {
    visionEnabled = false;
  }

  private final VisionConsumer consumer;
  private final VisionIO[] cameras;
  private final VisionIOInputsAutoLogged[] cameraInputs;
  private final String[] logKeys;
  private final Lock odometryLock;
  private final int numCameras;

  private final ArrayList<Pose3d> allRobotPoses = new ArrayList<>();
  private final ArrayList<Pose3d> allAcceptedPoses = new ArrayList<>();
  private final ArrayList<Pose3d> cameraRobotPoses = new ArrayList<>();
  private final ArrayList<Pose3d> cameraAcceptedPoses = new ArrayList<>();

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    cameras = io;

    numCameras = io.length;

    cameraInputs = new VisionIOInputsAutoLogged[numCameras];
    logKeys = new String[numCameras];
    for (int i = 0; i < numCameras; i++) {
      cameraInputs[i] = new VisionIOInputsAutoLogged();
      logKeys[i] = "Vision/" + CAMERA_NAMES[i];
    }

    odometryLock = VisionOdometry.getInstance().globalLock;
  }

  @Override
  public void periodic() {
    odometryLock.lock();

    try {
      for (int i = 0; i < numCameras; i++) {
        val inputs = cameraInputs[i];
        cameras[i].updateInputs(inputs);
        Logger.processInputs(logKeys[i], inputs);
      }
    } finally {
      odometryLock.unlock();
    }

    Logger.recordOutput("Vision/Enabled", visionEnabled);

    for (int cameraIdx = 0; cameraIdx < numCameras; cameraIdx++) {
      val inputs = cameraInputs[cameraIdx];
      // skip processing inputs if the camera is disconnected
      if (!inputs.connected) {
        continue;
      }

      cameraRobotPoses.clear();
      cameraAcceptedPoses.clear();

      for (var observation : inputs.poseObservations) {
        val tagsUsed = observation.tagsUsed();
        val pose = observation.pose();

        boolean reject =
            tagsUsed.length == 0
                || (tagsUsed.length == 1 && observation.averageAmbiguity() > maxAmbiguity)
                || Math.abs(pose.getZ()) > maxZError
                || pose.getX() < 0
                || pose.getX() > APRILTAG_LAYOUT.getFieldLength()
                || pose.getY() < 0
                || pose.getY() > APRILTAG_LAYOUT.getFieldWidth();

        cameraRobotPoses.add(pose);
        if (reject) {
          continue;
        }

        cameraAcceptedPoses.add(pose);

        double stdDevFactor = Math.pow(observation.averageDistance(), 2) / tagsUsed.length;
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;

        if (cameraIdx < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIdx];
          angularStdDev *= cameraStdDevFactors[cameraIdx];
        }

        if (visionEnabled) {
          consumer.addVisionPose(
              pose.toPose2d(),
              observation.timestamp(),
              VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
        }
      }

      Logger.recordOutput(
          logKeys[cameraIdx] + "/RobotPoses", cameraRobotPoses.toArray(Pose3d[]::new));
      Logger.recordOutput(
          logKeys[cameraIdx] + "/RobotPosesAccepted", cameraRobotPoses.toArray(Pose3d[]::new));

      allRobotPoses.addAll(cameraRobotPoses);
      allAcceptedPoses.addAll(cameraAcceptedPoses);
    }

    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allAcceptedPoses.toArray(Pose3d[]::new));
  }

  @FunctionalInterface
  public interface VisionConsumer {
    public void addVisionPose(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
  }
}
