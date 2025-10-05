package frc.robot.subsystems.end_effector;

import com.ctre.phoenix6.CANBus;
import frc.robot.Constants;

public final class EndEffectorConstants {
  public static final int kMotorID = 20, kExitBreakerID = 0, kEntranceBreakerID = 1;
  public static final CANBus kMotorBus = Constants.kRIOBus;
  public static final double kFastEjectSpeed = .9,
      kAutoEjectSpeed = 1,
      kSlowEjectSpeed = .25,
      kAlgaeBargeSpeed = 1,
      kAlgaeProcessorSpeed = 0.3,
      kIdleSpeed = .175,
      kIntakeSpeed = 0.1,
      kDriveBackSpeed = 0.075;

  // TODO: TUNE THIS VALUE (ASK MARK/BRENNAN)
  public static final double kSystemMOI = 0.02;
}
