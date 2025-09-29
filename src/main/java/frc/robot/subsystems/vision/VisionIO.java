package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface VisionIO {
  @AutoLog
  public class VisionIOInputs {
    public boolean connected = false;
    public PoseObservation[] poseObservations;
  }

  public final record PoseObservation(
      Pose3d pose,
      int[] tagsUsed,
      double averageDistance,
      double averageAmbiguity,
      double timestamp) {}

  public void updateInputs(VisionIOInputs inputs);

  public static final class VisionIOPhotonVision implements VisionIO {
    private final RegisteredCamera camera;

    public VisionIOPhotonVision(int cameraIdx, DoubleFunction<Pose2d> poseSupplier) {
      camera =
          VisionOdometry.getInstance()
              .registerCamera(CAMERA_NAMES[cameraIdx], ROBOT_TO_CAMERAS[cameraIdx], poseSupplier);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
      inputs.connected = camera.camera.isConnected();
      inputs.poseObservations = camera.poseObservations.toArray(PoseObservation[]::new);
      camera.clearCache();
    }
  }

  public final class VisionIOPhotonVisionSim implements VisionIO {
    private final RegisteredSimCamera camera;

    public VisionIOPhotonVisionSim(
        int cameraIdx,
        final DoubleFunction<Pose2d> poseSupplier,
        final Supplier<Pose2d> currentPose) {
      camera =
          VisionOdometry.getInstance()
              .registerSimCamera(
                  CAMERA_NAMES[cameraIdx], ROBOT_TO_CAMERAS[cameraIdx], poseSupplier, currentPose);
    }

    @Override
    public void updateInputs(final VisionIOInputs inputs) {
      inputs.connected = camera.camera.isConnected();
      inputs.poseObservations = camera.poseObservations.toArray(PoseObservation[]::new);
      camera.clearCache();
    }
  }
}
