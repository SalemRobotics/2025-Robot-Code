package frc.robot.subsystems;

import java.util.ArrayList;
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
    final AprilTagFieldLayout mFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    final PhotonCamera mCamera1 = new PhotonCamera(VisionConstants.kCamera1Name);
    final PhotonCamera mCamera2 = new PhotonCamera(VisionConstants.kCamera2Name);

    public ArrayList<VisionHelper> getCurrentPositionCam1(){
        var results = mCamera1.getAllUnreadResults();
        ArrayList<VisionHelper> returnPoses = new ArrayList<VisionHelper>();

        for(int i = 0; i < results.size(); i++){
            if (results.get(i).multitagResult.isPresent()) {
                Transform3d fieldToCamera = results.get(i).multitagResult.get().estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(VisionConstants.kRobotToCam1.inverse());
                returnPoses.add(new VisionHelper(
                    Optional.of(new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation()).toPose2d()),
                    results.get(i).getTimestampSeconds()
                ));
            } else if (!results.get(i).targets.isEmpty()){
                var target = results.get(i).targets.get(0);
                
                // Calculate robot pose
                var tagPose = mFieldLayout.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget =
                        new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(VisionConstants.kRobotToCam1.inverse());
                    returnPoses.add(new VisionHelper(
                        Optional.of(new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation()).toPose2d()),
                        results.get(i).getTimestampSeconds()
                    ));
                }
            }
        }
        return returnPoses;
    }

    public ArrayList<VisionHelper> getCurrentPositionCam2(){
        var results = mCamera2.getAllUnreadResults();
        ArrayList<VisionHelper> returnPoses = new ArrayList<VisionHelper>();
        for(int i = 0; i < results.size(); i++){
            if (results.get(i).multitagResult.isPresent()) {
                Transform3d fieldToCamera = results.get(i).multitagResult.get().estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(VisionConstants.kRobotToCam2.inverse());
                returnPoses.add(new VisionHelper(
                    Optional.of(new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation()).toPose2d()),
                    results.get(i).getTimestampSeconds()
                ));
            }
            else if(!results.get(i).targets.isEmpty()){
                var target = results.get(i).targets.get(0);
                
                // Calculate robot pose
                var tagPose = mFieldLayout.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget =
                        new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(VisionConstants.kRobotToCam2.inverse());
                    returnPoses.add( new VisionHelper(
                        Optional.of(new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation()).toPose2d()),
                        results.get(i).getTimestampSeconds()
                    ));
                }
            }
        }
        return returnPoses;
    }

}
