package frc.robot.autopilot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static frc.robot.FieldConstants.fieldCenter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.PositionUtils;
import java.util.HashMap;
import java.util.Map;

public enum Quadrant {
  Q1(
      AllianceFlipUtil.apply(
          new Pose2d(
              fieldCenter.getX() * 0.25, fieldCenter.getY() * 1.5, Rotation2d.fromDegrees(-60)))),
  Q2(
      AllianceFlipUtil.apply(
          new Pose2d(
              fieldCenter.getX() * 0.25, fieldCenter.getY() * 0.5, Rotation2d.fromDegrees(60)))),
  Q3(
      AllianceFlipUtil.apply(
          new Pose2d(
              fieldCenter.getX() * 0.75, fieldCenter.getY() * 0.5, Rotation2d.fromDegrees(120)))),
  Q4(
      AllianceFlipUtil.apply(
          new Pose2d(
              fieldCenter.getX() * 0.75, fieldCenter.getY() * 1.5, Rotation2d.fromDegrees(-120))));

  private final Pose2d m_middle;

  // allocate a HashMap for all of the pathfinds from one quadrant to another,
  private static final Map<Quadrant, Map<Quadrant, Command>> cachedPathfinds =
      new HashMap<>(4) {
        {
          put(Q1, new HashMap<>(3));
          put(Q2, new HashMap<>(3));
          put(Q3, new HashMap<>(3));
          put(Q4, new HashMap<>(3));
        }
      };
  private static final PathConstraints PATHFIND_CONSTRAINTS =
      new PathConstraints(
          MetersPerSecond.of(4.73), MetersPerSecondPerSecond.of(5),
          DegreesPerSecond.of(540), DegreesPerSecondPerSecond.of(720));
  private static boolean alreadInitializedPaths = false;

  private Quadrant(Pose2d mid) {
    m_middle = mid;
  }

  public Pose2d getM_middle() {
    return m_middle;
  }

  public Command pathfindTo(Quadrant other) {
    if (this == other)
      // we're already in the right place
      return Commands.none();
    else {
      var cmd = cachedPathfinds.get(this).get(other);

      if (cmd == null) {
        cmd = AutoBuilder.pathfindToPose(other.m_middle, PATHFIND_CONSTRAINTS);
      }

      return cmd;
    }
  }

  public Command driveTo(Drive drive, Quadrant to, double positionTolerance) {
    return pathfindTo(to)
        .until(() -> PositionUtils.getDistance(drive, to.m_middle) <= positionTolerance);
  }

  /**
   * Initializes every path between different quadrants preemptively. This should not be run before
   * the paths should be used, especially on the main thread.
   *
   * <p>This is, by default, run when the robot is first disabled, and is a no-op if run repeatedly.
   */
  public static void initializeAllPaths() {
    if (!alreadInitializedPaths) {
      cachedPathfinds.forEach(
          (origin, map) -> {
            if (map.size() == 3) return;

            for (Quadrant other : Quadrant.values()) {
              if (other != origin) {
                var waypoints = PathPlannerPath.waypointsFromPoses(origin.m_middle, other.m_middle);
                var path =
                    new PathPlannerPath(
                        waypoints,
                        PATHFIND_CONSTRAINTS,
                        new IdealStartingState(0, origin.m_middle.getRotation()),
                        new GoalEndState(0, other.m_middle.getRotation()));
                map.put(other, AutoBuilder.followPath(path));
              }
            }
          });

      alreadInitializedPaths = true;
    }
  }
}
