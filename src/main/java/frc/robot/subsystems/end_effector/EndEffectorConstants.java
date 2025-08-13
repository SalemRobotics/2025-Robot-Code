package frc.robot.subsystems.end_effector;

import com.ctre.phoenix6.CANBus;

public final class EndEffectorConstants {
  public static final int kMotorID = 20, kExitBreakerID = 0, kEntranceBreakerID = 1;
  public static final CANBus kMotorBus = new CANBus("rio");
  public static final double kFastEjectSpeed = .9,
      kAutoEjectSpeed = 1,
      kSlowEjectSpeed = .3,
      kAlgaeBargeSpeed = 1,
      kAlgaeProcessorSpeed = 0.3,
      kIdleSpeed = .25,
      kIntakeSpeed = 0.125,
      kDriveBackSpeed = 0.1;

  // TODO: TUNE THIS VALUE (ASK MARK/BRENNAN)
  public static final double kSystemMOI = 0.02;
}
