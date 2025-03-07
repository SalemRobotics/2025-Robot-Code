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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.AllianceFlipUtil;

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
    public static final Translation2d kReefCenter = new Translation2d(4.48945, kFieldWidth/2);

    public static Pose2d[] blueCenterFaces =
      new Pose2d[6]; // Starting facing away from the driver station in clockwise order
    public static Pose2d[] redCenterFaces = 
      new Pose2d[6];

    static {
      // Initialize faces
      blueCenterFaces[3] =
          new Pose2d(
              Units.inchesToMeters(144.003),
              Units.inchesToMeters(158.500),
              Rotation2d.fromDegrees(180));
      blueCenterFaces[4] =
          new Pose2d(
              Units.inchesToMeters(160.373),
              Units.inchesToMeters(186.857),
              Rotation2d.fromDegrees(120));
      blueCenterFaces[5] =
          new Pose2d(
              Units.inchesToMeters(193.116),
              Units.inchesToMeters(186.858),
              Rotation2d.fromDegrees(60));
      blueCenterFaces[0] =
          new Pose2d(
              Units.inchesToMeters(209.489),
              Units.inchesToMeters(158.502),
              Rotation2d.fromDegrees(0));
      blueCenterFaces[1] =
          new Pose2d(
              Units.inchesToMeters(193.118),
              Units.inchesToMeters(130.145),
              Rotation2d.fromDegrees(-60));
      blueCenterFaces[2] =
          new Pose2d(
              Units.inchesToMeters(160.375),
              Units.inchesToMeters(130.144),
              Rotation2d.fromDegrees(-120));
      
      for (int i = 0; i < 6; i++) {
        redCenterFaces[i] = AllianceFlipUtil.applyUnchecked(blueCenterFaces[i]);
      }
    }
    public static Map<Integer, Map<Boolean, Pose2d>> kScoringPoses = new HashMap<Integer, Map<Boolean, Pose2d>>();

    static{
      Pose2d startingPoseRight = new Pose2d(3.098, 3.851, Rotation2d.fromDegrees(0));
      Pose2d startingPoseLeft = new Pose2d(3.098, 4.201, Rotation2d.fromDegrees(0));
      for(int i = 0; i < 6; i++){
        
        Pose2d currentPoseLeft = startingPoseLeft.rotateAround(kReefCenter, Rotation2d.fromDegrees(60 * i));
        Pose2d currentPoseRight = startingPoseRight.rotateAround(kReefCenter, Rotation2d.fromDegrees(60 * i));

        SmartDashboard.putString("Pose " + i + " Left", "X: " + currentPoseLeft.getX() + 
          ", Y: " + currentPoseLeft.getY() + ", Heading: " +
          currentPoseLeft.getRotation());
          SmartDashboard.putString("Pose " + i + " Right", "X: " + currentPoseRight.getX() + 
          ", Y: " + currentPoseRight.getY() + ", Heading: " +
          currentPoseRight.getRotation());
      
      }
      kScoringPoses.put(0, new HashMap<Boolean, Pose2d>());
      kScoringPoses.get(0).put(false, new Pose2d(5.881, 3.851, Rotation2d.fromDegrees(180))); //AL
      kScoringPoses.get(0).put(true, new Pose2d(5.881, 4.201, Rotation2d.fromDegrees(180))); //AR
      kScoringPoses.put(1, new HashMap<Boolean, Pose2d>());
      kScoringPoses.get(1).put(false, new Pose2d(5.034, 2.733, Rotation2d.fromDegrees(120))); //BL
      kScoringPoses.get(1).put(true, new Pose2d(5.337, 2.908, Rotation2d.fromDegrees(120))); //BR
      kScoringPoses.put(2, new HashMap<Boolean, Pose2d>());
      kScoringPoses.get(2).put(false, new Pose2d(3.642, 2.908, Rotation2d.fromDegrees(60))); //CL
      kScoringPoses.get(2).put(true, new Pose2d(3.945, 2.733, Rotation2d.fromDegrees(60))); //CR
      kScoringPoses.put(3, new HashMap<Boolean, Pose2d>());
      kScoringPoses.get(3).put(false, new Pose2d(3.098, 4.201, Rotation2d.fromDegrees(0))); //DL
      kScoringPoses.get(3).put(true, new Pose2d(3.098, 3.851, Rotation2d.fromDegrees(0))); //DR
      kScoringPoses.put(4, new HashMap<Boolean, Pose2d>());
      kScoringPoses.get(4).put(false, new Pose2d(3.945, 5.318, Rotation2d.fromDegrees(300))); //EL
      kScoringPoses.get(4).put(true, new Pose2d(3.642, 5.143, Rotation2d.fromDegrees(300))); //ER
      kScoringPoses.put(5, new HashMap<Boolean, Pose2d>());
      kScoringPoses.get(5).put(false, new Pose2d(5.337, 5.143, Rotation2d.fromDegrees(240))); //FL
      kScoringPoses.get(5).put(true, new Pose2d(5.034, 5.318, Rotation2d.fromDegrees(240))); //FR
    }
    public static final Pose2d kCoral1PoseLeft = new Pose2d(1.65, 0.72, Rotation2d.fromDegrees(55));
    public static final Pose2d kCoral1PoseRight = new Pose2d(1.11, 1.10, Rotation2d.fromDegrees(55));
    public static final Pose2d kCoral2PoseRight = new Pose2d(1.65, 7.50, Rotation2d.fromDegrees(305));
    public static final Pose2d kCoral2PoseLeft = new Pose2d(1.13, 7.04, Rotation2d.fromDegrees(305));
    

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
