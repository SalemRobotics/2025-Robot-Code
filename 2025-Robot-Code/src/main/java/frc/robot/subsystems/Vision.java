package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.VisionHelper;

public class Vision extends SubsystemBase{
    final AprilTagFieldLayout mFieldLayout;
    final PhotonCamera mCamera1;
    final PhotonCamera mCamera2;

    public Vision() {
        mCamera1 = new PhotonCamera(VisionConstants.kCamera1Name);
        mCamera2 = new PhotonCamera(VisionConstants.kCamera1Name);
        mFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    }

    public VisionHelper getCurrentPositionCam1(){
        var result = mCamera1.getLatestResult();
        if (result.multitagResult.isPresent()) {
            Transform3d fieldToCamera = result.multitagResult.get().estimatedPose.best;
            Transform3d fieldToRobot = fieldToCamera.plus(VisionConstants.kRobotToCam1.inverse());
            return new VisionHelper(
                Optional.of(new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation()).toPose2d()),
                result.getTimestampSeconds()
            );
        }
        else if(!result.targets.isEmpty()){
            var target = result.targets.get(0);
            
            // Calculate robot pose
            var tagPose = mFieldLayout.getTagPose(target.fiducialId);
            if (tagPose.isPresent()) {
                Transform3d fieldToTarget =
                    new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
                Transform3d cameraToTarget = target.bestCameraToTarget;
                Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                Transform3d fieldToRobot = fieldToCamera.plus(VisionConstants.kRobotToCam1.inverse());
                return new VisionHelper(
                    Optional.of(new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation()).toPose2d()),
                    result.getTimestampSeconds()
                );
            }
        }
        return new VisionHelper(Optional.empty(), 0.0);
        
    }

    public VisionHelper getCurrentPositionCam2(){
        var result = mCamera2.getLatestResult();
        if (result.multitagResult.isPresent()) {
            Transform3d fieldToCamera = result.multitagResult.get().estimatedPose.best;
            Transform3d fieldToRobot = fieldToCamera.plus(VisionConstants.kRobotToCam2.inverse());
            return new VisionHelper(
                Optional.of(new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation()).toPose2d()),
                result.getTimestampSeconds()
            );
        }
        else if(!result.targets.isEmpty()){
            var target = result.targets.get(0);
            
            // Calculate robot pose
            var tagPose = mFieldLayout.getTagPose(target.fiducialId);
            if (tagPose.isPresent()) {
                Transform3d fieldToTarget =
                    new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
                Transform3d cameraToTarget = target.bestCameraToTarget;
                Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                Transform3d fieldToRobot = fieldToCamera.plus(VisionConstants.kRobotToCam2.inverse());
                return new VisionHelper(
                    Optional.of(new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation()).toPose2d()),
                    result.getTimestampSeconds()
                );
            }
        }
        return new VisionHelper(Optional.empty(), 0.0);
    }

}
