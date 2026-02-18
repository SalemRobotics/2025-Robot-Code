package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import lombok.val;

public final class DeltaTimeCalculator {
  private double lastTimestamp = RobotController.getFPGATime() / 1e6;

  public double calculate() {
    val newTimestamp = RobotController.getFPGATime() / 1e6;
    val delta = newTimestamp - lastTimestamp;
    lastTimestamp = newTimestamp;

    return delta;
  }
}
