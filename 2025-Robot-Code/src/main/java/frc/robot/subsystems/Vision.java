package frc.robot.subsystems;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.VisionHelper;

public class Vision extends SubsystemBase{
    static AprilTagFieldLayout mFieldLayout;
    static boolean useCustomField = false;
    static{
        try {
            mFieldLayout = new AprilTagFieldLayout(Path.of(Filesystem.getDeployDirectory().getAbsolutePath() + "/weldedlayout.json"));
            useCustomField = true;
        } catch (Exception e) {
            // TODO: handle exception
            mFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        }
        SmartDashboard.putBoolean("Field Config", useCustomField);
    }
    final PhotonCamera mCamera1 = new PhotonCamera(VisionConstants.kCamera1Name);
    final PhotonCamera mCamera2 = new PhotonCamera(VisionConstants.kCamera2Name);


    public ArrayList<VisionHelper> getVisionUpdates() {
        ArrayList<VisionHelper> poses = new ArrayList<>();

        var results = mCamera1.getAllUnreadResults();

        for (var result : results) {
            var multitag = result.multitagResult;
            Transform3d fieldToCamera = null;

            if (multitag.isPresent()) {
                fieldToCamera = multitag.get().estimatedPose.best;
            } else if (result.targets.isEmpty()) 
                continue;
            else {
                var target = result.targets.get(0);
                var tagPoseOptional = mFieldLayout.getTagPose(target.fiducialId);
                if (tagPoseOptional.isEmpty())
                    continue;
                
                Pose3d tagPose = tagPoseOptional.get();
                Transform3d fieldToTarget = new Transform3d(tagPose.getTranslation(), tagPose.getRotation());
                Transform3d cameraToTarget = target.bestCameraToTarget;
                fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
            }

            Transform3d fieldToRobot = fieldToCamera.plus(VisionConstants.kRobotToCam1.inverse());
            Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
            poses.add(new VisionHelper(robotPose.toPose2d(), result.getTimestampSeconds()));
        }

        results = mCamera2.getAllUnreadResults();

        for (var result : results) {
            var multitag = result.multitagResult;
            Transform3d fieldToCamera = null;

            if (multitag.isPresent()) {
                fieldToCamera = multitag.get().estimatedPose.best;
            } else if (result.targets.isEmpty()) 
                continue;
            else {
                var target = result.targets.get(0);
                var tagPoseOptional = mFieldLayout.getTagPose(target.fiducialId);
                if (tagPoseOptional.isEmpty())
                    continue;
                
                Pose3d tagPose = tagPoseOptional.get();
                Transform3d fieldToTarget = new Transform3d(tagPose.getTranslation(), tagPose.getRotation());
                Transform3d cameraToTarget = target.bestCameraToTarget;
                fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
            }

            Transform3d fieldToRobot = fieldToCamera.plus(VisionConstants.kRobotToCam2.inverse());
            Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
            poses.add(new VisionHelper(robotPose.toPose2d(), result.getTimestampSeconds()));
        }

        return poses;
    }

    public ArrayList<VisionHelper> getCurrentPositionCam1(){
        var results = mCamera1.getAllUnreadResults();
        ArrayList<VisionHelper> returnPoses = new ArrayList<VisionHelper>();

        for(int i = 0; i < results.size(); i++){
            if (results.get(i).multitagResult.isPresent()) {
                Transform3d fieldToCamera = results.get(i).multitagResult.get().estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(VisionConstants.kRobotToCam1.inverse());
                returnPoses.add(new VisionHelper(
                    (new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation()).toPose2d()),
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
                        new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation()).toPose2d(),
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
                    new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation()).toPose2d(),
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
                        new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation()).toPose2d(),
                        results.get(i).getTimestampSeconds()
                    ));
                }
            }
        }
        return returnPoses;
    }

}
