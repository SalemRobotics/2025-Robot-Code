// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class ElevatorConstants {
    public static final double kStowedHeight = 0;
    public static final double kL1Height = 1.0;
    public static final double kL2Height = 2.0;
    public static final double kL3Height = 3.0;
    public static final double kL4Height = 4.0;
    public static final CANBus kElevatorMotorBus = new CANBus("canivore");
    public static final int kElevatorMotorAPort = 13;
    public static final int kElevatorMotorBPort = 14;
    public static final double kSensorToMechanismRatio = 9.0;
    public static final double kElevatorMaxSpeed = 5.0;
    public static final double kElevatorMaxAcceleration = 10.0;
    public static final double kElevatorMaxJerk = 100.0;
    public static final double kElevatorG = 0.25;
    public static final double kElevatorS = 0.25;
    public static final double kElevatorV = 0.12;
    public static final double kElevatorA = 0.01;
    public static final double kElevatorP = 60.0;
    public static final double kElevatorI = 0.0;
    public static final double kElevatorD = 0.5;
  }

  public static class VisionConstants {
    public static String kCamera1Name = "";
    public static String kCamera2Name = "";
  }

  public static class AlgaeConstants {
    public static final int kAlgaeMotorPort = 20;
    public static final CANBus kAlgaeMotorBus = new CANBus("canivore");
    public static final double kAlgaeMotorSpeed = 1.0;
  }
}
