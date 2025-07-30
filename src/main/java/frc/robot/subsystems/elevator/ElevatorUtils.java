package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.kSpoolCircumference;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public final class ElevatorUtils {
  public static Distance rotsToInches(Angle rotations) {
    return Inches.of(rotations.in(Rotations) * kSpoolCircumference.in(Inches));
  }

  public static Angle inchesToRotations(Distance inches) {
    return Rotations.of(inches.in(Inches) / kSpoolCircumference.in(Inches));
  }

  public static AngularVelocity convertVelocity(LinearVelocity velocity) {
    return RotationsPerSecond.of(velocity.in(InchesPerSecond) / kSpoolCircumference.in(Inches));
  }
}
