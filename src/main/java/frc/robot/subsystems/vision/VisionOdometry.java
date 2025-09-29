package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Hertz;
import static frc.robot.subsystems.vision.VisionConstants.CAMERA_FPS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.ParallelizedOdometry;
import frc.robot.util.ParallelizedOdometry.OdometryTask;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;
import lombok.val;

final class VisionOdometry implements OdometryTask {
  private static VisionOdometry instance = null;

  public static VisionOdometry getInstance() {
    if (instance == null) {
      instance = new VisionOdometry();
    }

    return instance;
  }

  public final Lock globalLock = new ReentrantLock(true);
  private final List<RegisteredCamera> cameras = new ArrayList<>(2);

  public void start() {
    ParallelizedOdometry.register(this, Hertz.of(CAMERA_FPS));
  }

  public RegisteredCamera registerCamera(
      String name, Transform3d robotToCamera, DoubleFunction<Pose2d> poseSupplier) {
    val registered = new RegisteredCamera(name, robotToCamera, poseSupplier);
    cameras.add(registered);

    return registered;
  }

  public RegisteredSimCamera registerSimCamera(
      String name,
      Transform3d robotToCamera,
      DoubleFunction<Pose2d> poseSupplier,
      Supplier<Pose2d> currentPose) {
    val registered = new RegisteredSimCamera(name, robotToCamera, poseSupplier, currentPose);
    cameras.add(registered);

    return registered;
  }

  @Override
  public void updateOdometry() {
    globalLock.lock();
    try {
      cameras.forEach(RegisteredCamera::update);
    } finally {
      globalLock.unlock();
    }
  }
}
