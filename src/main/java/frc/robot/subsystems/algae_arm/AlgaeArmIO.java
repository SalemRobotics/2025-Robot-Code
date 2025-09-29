package frc.robot.subsystems.algae_arm;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface AlgaeArmIO {
  @AutoLog
  public static class AlgaeArmIOInputs {
    public boolean connected = false;
    public Angle position = Rotations.zero();
    public AngularVelocity velocity = RotationsPerSecond.zero();
    public Angle setpoint = Rotations.zero();
  }

  public void updateInputs(AlgaeArmIOInputs inputs);

  public default void deploy() {}

  public default void stow() {}
}
