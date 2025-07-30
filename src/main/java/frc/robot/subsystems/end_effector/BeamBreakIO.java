package frc.robot.subsystems.end_effector;

import org.littletonrobotics.junction.AutoLog;

public interface BeamBreakIO {
  public default boolean isBroken() {
    return false;
  }

  public default void updateInputs(BeamBreakIOInputs inputs) {}

  @AutoLog
  public static class BeamBreakIOInputs {
    boolean isBroken = false;
  }
}
