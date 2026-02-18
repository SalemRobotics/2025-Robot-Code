// Copyright (c) 2025 FRC 6324 The Blue Devils

package frc.robot.util;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.LocalADStar;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import lombok.val;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * A helper class which wrapps pathplanner's AD* ({@link LocalADStar}) implementation with logging
 * enabled for use with AdvantageKit's log replay. Users not using the AdvantageKit logging
 * framework should not use this class; instead, use the regular {@link LocalADStar}
 */
public class LocalADStarAK extends LocalADStar implements LoggableInputs {
  private boolean isNewPathAvailable = false;
  private List<PathPoint> currentPathPoints = Collections.emptyList();

  /**
   * Get if a new path has been calculated since the last time a path was retrieved
   *
   * @return True if a new path is available
   */
  @Override
  public boolean isNewPathAvailable() {
    if (!Logger.hasReplaySource()) {
      isNewPathAvailable = super.isNewPathAvailable();
    }

    Logger.processInputs("LocalADStarAK", this);
    return isNewPathAvailable;
  }

  /**
   * Get the most recently calculated path
   *
   * @param constraints The path constraints to use when creating the path
   * @param goalEndState The goal end state to use when creating the path
   * @return The PathPlannerPath created from the points calculated by the pathfinder
   */
  @Override
  public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
    if (!Logger.hasReplaySource()) {
      PathPlannerPath currentPath = super.getCurrentPath(constraints, goalEndState);

      if (currentPath != null) {
        currentPathPoints = currentPath.getAllPathPoints();
      } else {
        currentPathPoints = Collections.emptyList();
      }
    }

    Logger.processInputs("LocalADStarAK", this);

    if (currentPathPoints.isEmpty()) {
      return null;
    }

    return PathPlannerPath.fromPathPoints(currentPathPoints, constraints, goalEndState);
  }

  /**
   * Set the start position to pathfind from
   *
   * @param startPosition Start position on the field. If this is within an obstacle it will be
   *     moved to the nearest non-obstacle node.
   */
  @Override
  public void setStartPosition(Translation2d startPosition) {
    if (!Logger.hasReplaySource()) {
      super.setStartPosition(startPosition);
    }
  }

  /**
   * Set the goal position to pathfind to
   *
   * @param goalPosition Goal position on the field. f this is within an obstacle it will be moved
   *     to the nearest non-obstacle node.
   */
  @Override
  public void setGoalPosition(Translation2d goalPosition) {
    if (!Logger.hasReplaySource()) {
      super.setGoalPosition(goalPosition);
    }
  }

  /**
   * Set the dynamic obstacles that should be avoided while pathfinding.
   *
   * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d represents
   *     opposite corners of a bounding box.
   * @param currentRobotPos The current position of the robot. This is needed to change the start
   *     position of the path to properly avoid obstacles
   */
  @Override
  public void setDynamicObstacles(
      List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
    if (!Logger.hasReplaySource()) {
      super.setDynamicObstacles(obs, currentRobotPos);
    }
  }

  @Override
  public void toLog(LogTable table) {
    table.put("IsNewPathAvailable", isNewPathAvailable);

    val pointsLogged = new double[currentPathPoints.size() * 2];
    int idx = 0;
    for (val point : currentPathPoints) {
      pointsLogged[idx] = point.position.getX();
      pointsLogged[idx + 1] = point.position.getY();
      idx += 2;
    }

    table.put("CurrentPathPoints", pointsLogged);
  }

  @Override
  public void fromLog(LogTable table) {
    isNewPathAvailable = table.get("IsNewPathAvailable", false);
    val pointsLogged = table.get("CurrentPathPoints", new double[0]);

    List<PathPoint> pathPoints = new ArrayList<>(pointsLogged.length / 2);
    for (int i = 0; i < pointsLogged.length; i += 2) {
      pathPoints.add(new PathPoint(new Translation2d(pointsLogged[i], pointsLogged[i + 1]), null));
    }

    currentPathPoints = pathPoints;
  }
}
