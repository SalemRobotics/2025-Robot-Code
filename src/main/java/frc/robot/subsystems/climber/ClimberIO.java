package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public boolean connected;

    public AngularVelocity motorSpeed = RotationsPerSecond.zero();
    public Angle servoSetpoint = Degrees.zero();
  }

  public void updateInputs(ClimberIOInputs inputs);

  public default void deploy() {}

  public default void retract() {}

  public default void stop() {}
}
