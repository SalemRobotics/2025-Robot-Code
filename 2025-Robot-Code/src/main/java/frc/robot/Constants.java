// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation3d;

import edu.wpi.first.math.geometry.Transform3d;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;


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
    public static final double kRumbleStrength = 0.5;
  }

  public static class ElevatorConstants {
    /*
     * 1 rotation = ~6.426 in of carriage travel
     * more precise: 1 rotation = ~6.425574853 inches, obtained by CAD & 2πr with 
     *  r = 1.022662 and r is the radius of the sprocket
     * 
     * Our current positions *work*, but finding exact heights in inches and getting rotations with this rot:in ratio 
     * may be more precise
     */
    public static final double kStowedHeight = 0;
    public static final double kL1Height = 1.1;
    public static final double kL2Height = 1.67;
    public static final double kL3Height = 2.82;
    public static final double kL4Height = 4.69;
    public static final CANBus kElevatorMotorBus = new CANBus("canivore0");
    public static final int kElevatorMotorAPort = 13;
    public static final int kElevatorMotorBPort = 14;
    public static final double kSensorToMechanismRatio = 9.0;
    public static final double kElevatorMaxSpeed = 9.0;
    
    public static final double kElevatorMaxAcceleration = 13.0;
    public static final double kElevatorMaxJerk = 100.0;
    public static final double kElevatorG = 0.25;
    public static final double kElevatorS = 0.25;
    public static final double kElevatorV = 0.12;
    public static final double kElevatorA = 0.01;
    public static final double kElevatorP = 60.0;
    public static final double kElevatorI = 0.0;
    public static final double kElevatorD = 0.5;
  }

  public static class DriveConstants {
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidthMeters = Units.inchesToMeters(23.72921);
    // Distance between front and back wheels on robot
    public static final double kWheelBaseMeters = Units.inchesToMeters(23.72921);

    public static final PathConstraints kScoringConstraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);

    public static final PathConstraints kMobilityConstraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
  }

  public static class AutoConstants {
    public static final double kTranslationP = 3;
    public static final double kTranslationI = 0;
    public static final double kTranslationD = 0;

    public static final double kRotationP = 5.5;
    public static final double kRotationI = 0;
    public static final double kRotationD = 0;

    /** Map of folder names to lists of auto command names */
    public static final HashMap<String, List<String>> kAutoFolders = new HashMap<>() {{
        put("Test Autos", List.of(
          "Test Auto",
          "Test Auto 2",
          "Test Auto 3"
          ));

        put("Basic Autos", List.of(
          "Do Nothing",
          "1"
        ));
    }};
  }

  public static class VisionConstants {
    public static String kCamera1Name = "Left";
    public static String kCamera2Name = "Right";

    public static final Transform3d kRobotToCam1 =
      new Transform3d(
        new Translation3d(Units.inchesToMeters(11.58), Units.inchesToMeters(11.189), Units.inchesToMeters(8.25)), 
        new Rotation3d(0, Units.degreesToRadians(-15.0), Units.degreesToRadians(-20.0)));

    public static final Transform3d kRobotToCam2 =
      new Transform3d(
        new Translation3d(Units.inchesToMeters(11.58), Units.inchesToMeters(-11.189), Units.inchesToMeters(8.25)), 
        new Rotation3d(0, Units.degreesToRadians(-15.0), Units.degreesToRadians(15.0)));
  }

  public static class AlgaeConstants {
    public static final int kAlgaeMotorPort = 21;
    public static final double kAlgaeMotorSpeed = 1.0;
    public static final double kAlgaeExtendedRotation = 0.25;
    public static final double kAlgaeStowedRotation = 0;
    public static final double kSensorToMechanismRatio = 25.0;
  }

  public static class EndEffectorConstants {
    public static final int kMotorPort = 20;
    public static final int kEntranceBreakerPort = 1;
    public static final int kExitBreakerPort = 0;

    public static final double kFastEjectSpeed = 0.9;
    public static final double kDefaultEjectSpeed = 0.6;
    public static final double kSlowEjectSpeed = 0.3;

    public static final double kIdleSpeed = 0.15;
    public static final double kIntakeSpeed = 0.06;
    
    public static final double kSensorToMechanismRatio = 25.0;
  }
}
