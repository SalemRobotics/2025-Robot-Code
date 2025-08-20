package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ServoIO {
  @AutoLog
  public static class ServoIOInputs {
    public double angle = 0;
  }

  public default void setAngle(double angle) {}

  public default void updateInputs(ServoIOInputs inputs) {}
}
