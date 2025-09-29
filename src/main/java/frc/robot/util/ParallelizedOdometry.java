package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

public final class ParallelizedOdometry {
  private static final ThreadGroup THREAD_GROUP = new ThreadGroup("OdometryThreadGroup");
  private static int THREAD_ID = 0;
  private static final Map<OdometryTask, ScheduledFuture<?>> scheduledOdometry =
      new ConcurrentHashMap<>();
  private static final ScheduledExecutorService workPool =
      Executors.newScheduledThreadPool(
          1,
          action -> {
            Thread thread = new Thread(THREAD_GROUP, action, "OdometryThread" + THREAD_ID++);
            thread.setDaemon(true);

            thread.setUncaughtExceptionHandler(
                (t, exception) -> {
                  DriverStation.reportError(
                      "Uncaught error in odometry threads: " + exception.getMessage(),
                      exception.getStackTrace());
                });

            return thread;
          });

  @FunctionalInterface
  public interface OdometryTask {
    public void updateOdometry();
  }

  public static ScheduledFuture<?> register(OdometryTask task, Frequency freq) {
    var future =
        workPool.scheduleAtFixedRate(
            task::updateOdometry,
            0,
            (long) (freq.asPeriod().in(Microseconds)),
            TimeUnit.MICROSECONDS);
    scheduledOdometry.put(task, future);

    return future;
  }
}
