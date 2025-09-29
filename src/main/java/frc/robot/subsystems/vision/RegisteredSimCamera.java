package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

final class RegisteredSimCamera extends RegisteredCamera {
  private static VisionSystemSim system = null;

  private static VisionSystemSim getSystem() {
    if (system == null) {
      system = new VisionSystemSim("main");
      system.addAprilTags(APRILTAG_LAYOUT);
    }

    return system;
  }

  private final PhotonCameraSim cameraSim;
  private final Supplier<Pose2d> currentPose;

  /**
   * Constructs a new simulated camera that is registered by {@link VisionOdometry}
   *
   * @param name The name of the camera
   * @param robotToCamera The transform that converts a camera-relative pose to a robot-relative
   *     pose
   * @param robotPose A function that gets the robot's pose at a given time
   * @param getCurrentPose Gets the <i>current</i> robot pose, for updating the vision system
   *     simulation.
   */
  protected RegisteredSimCamera(
      String name,
      Transform3d robotToCamera,
      DoubleFunction<Pose2d> robotPose,
      Supplier<Pose2d> getCurrentPose) {
    super(name, robotToCamera, robotPose);
    this.currentPose = getCurrentPose;

    SimCameraProperties props = new SimCameraProperties();
    props.setAvgLatencyMs(30);
    props.setFPS(CAMERA_FPS);
    props.setLatencyStdDevMs(4);

    cameraSim = new PhotonCameraSim(camera, props);
    getSystem().addCamera(cameraSim, robotToCamera);
  }

  @Override
  protected void update() {
    getSystem().update(currentPose.get());

    super.update();
  }
}
