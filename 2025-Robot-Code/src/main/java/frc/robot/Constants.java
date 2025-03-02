// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
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

  public static class FieldConstants {
    public static final double kFieldLength = 17.5482504;
    public static final double kFieldWidth = 8.0518;

    public static final Pose2d[] centerFaces =
      new Pose2d[6]; // Starting facing away from the driver station in clockwise order

    static {
      // Initialize faces
      centerFaces[3] =
          new Pose2d(
              Units.inchesToMeters(144.003),
              Units.inchesToMeters(158.500),
              Rotation2d.fromDegrees(180));
      centerFaces[4] =
          new Pose2d(
              Units.inchesToMeters(160.373),
              Units.inchesToMeters(186.857),
              Rotation2d.fromDegrees(120));
      centerFaces[5] =
          new Pose2d(
              Units.inchesToMeters(193.116),
              Units.inchesToMeters(186.858),
              Rotation2d.fromDegrees(60));
      centerFaces[0] =
          new Pose2d(
              Units.inchesToMeters(209.489),
              Units.inchesToMeters(158.502),
              Rotation2d.fromDegrees(0));
      centerFaces[1] =
          new Pose2d(
              Units.inchesToMeters(193.118),
              Units.inchesToMeters(130.145),
              Rotation2d.fromDegrees(-60));
      centerFaces[2] =
          new Pose2d(
              Units.inchesToMeters(160.375),
              Units.inchesToMeters(130.144),
              Rotation2d.fromDegrees(-120));
    }
    public static Map<Integer, Map<Boolean, Pose2d>> kScoringPoses = new HashMap<Integer, Map<Boolean, Pose2d>>();

    static{
      kScoringPoses.put(0, new HashMap<Boolean, Pose2d>());
      kScoringPoses.get(0).put(false, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))); //AL
      kScoringPoses.get(0).put(true, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))); //AR
      kScoringPoses.put(1, new HashMap<Boolean, Pose2d>());
      kScoringPoses.get(1).put(false, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))); //BL
      kScoringPoses.get(1).put(true, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))); //BR
      kScoringPoses.put(2, new HashMap<Boolean, Pose2d>());
      kScoringPoses.get(2).put(false, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))); //CL
      kScoringPoses.get(2).put(true, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))); //CR
      kScoringPoses.put(3, new HashMap<Boolean, Pose2d>());
      kScoringPoses.get(3).put(false, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))); //DL
      kScoringPoses.get(3).put(true, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))); //DR
      kScoringPoses.put(4, new HashMap<Boolean, Pose2d>());
      kScoringPoses.get(4).put(false, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))); //EL
      kScoringPoses.get(4).put(true, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))); //ER
      kScoringPoses.put(5, new HashMap<Boolean, Pose2d>());
      kScoringPoses.get(5).put(false, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))); //FL
      kScoringPoses.get(5).put(true, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))); //FR
    }
    public static final Pose2d kCoral1Pose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
    public static final Pose2d kCoral2Pose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
    

  }

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
    public static final double kL2Height = 1.536;
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
    public static final double kTranslationP = 10;
    public static final double kTranslationI = 0;
    public static final double kTranslationD = 0;

    public static final double kRotationP = 7;
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
          "Mobility Auto"
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

    public static final double kIdleSpeed = 0.2;
    public static final double kAdjustSpeed = 0.08;
    
    public static final double kSensorToMechanismRatio = 25.0;
    public static final double kDelayPeriod = 0.20;
  }
}
