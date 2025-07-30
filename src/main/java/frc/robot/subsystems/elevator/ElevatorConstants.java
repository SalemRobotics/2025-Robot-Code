package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.*;

public final class ElevatorConstants {
  public static final int kLeaderId = 13, kFollowerId = 14;
  public static final CANBus kMotorBus = new CANBus("canivore0");

  public static final class Trapezoidal {
    public static final AngularVelocity kMaxVelocity = RotationsPerSecond.of(15);
    public static final AngularAcceleration kMaxAcceleration = RotationsPerSecondPerSecond.of(35);
    public static final Velocity<AngularAccelerationUnit> kMaxJerk =
        RotationsPerSecondPerSecond.per(Second).of(100);
  }

  public static final class Exponential {
    public static final AngularAcceleration kMaxAcceleration = RotationsPerSecondPerSecond.of(50);
    public static final Velocity<AngularAccelerationUnit> kMaxJerk =
        RotationsPerSecondPerSecond.per(Second).of(200);

    public static final double kExpo_kV = 0.6;
    public static final double kExpo_kA = 0.2;
  }

  /** Gains for moving the elevator to a setpoint from stow */
  public static final Slot0Configs kRiseGains =
      new Slot0Configs()
          .withGravityType(GravityTypeValue.Elevator_Static)
          .withKP(50)
          .withKI(0)
          .withKD(30)
          .withKV(2.33)
          .withKA(0.1)
          .withKG(1.0)
          .withKS(0.2);

  /** Gains for stowing the elevator */
  public static final Slot1Configs kStowGains =
      new Slot1Configs()
          .withGravityType(GravityTypeValue.Elevator_Static)
          .withKP(25)
          .withKI(0)
          .withKD(5)
          .withKV(2.33)
          .withKA(0.05)
          .withKG(0.8)
          .withKS(0.2);

  /** Gains for traversing between any two non-stow setpoints */
  public static final Slot2Configs kTraversalGains =
      new Slot2Configs()
          .withGravityType(GravityTypeValue.Elevator_Static)
          .withKP(50)
          .withKI(0)
          .withKD(3)
          .withKV(2.33)
          .withKA(0.07)
          .withKG(0.72)
          .withKS(0.2);

  public static final Distance kSpoolCircumference = Inches.of(6);
}
