package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase{
    final AprilTagFieldLayout mFieldLayout;
    final PhotonCamera mCamera1;
    final PhotonCamera mCamera2;
    final PhotonPoseEstimator poseEstimator;

    Vision() {
        mCamera1 = new PhotonCamera(VisionConstants.kCamera1Name);
        mCamera2 = new PhotonCamera(VisionConstants.kCamera1Name);
        mFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        poseEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToCam1);

    }

    /*public Transform3d getCurrentPositionCam1(){
        var result = mCamera1.getLatestResult();
        if (result.getMultiTagResult().estimatedPose.isPresent) {
        Transform3d fieldToCamera = result.getMultiTagResult().estimatedPose.best;
        }
    }

    public Transform3d getCurrentPositionCam2(){
        var result = mCamera1.getLatestResult();
        if (result.getMultiTagResult().estimatedPose.isPresent) {
        Transform3d fieldToCamera = result.getMultiTagResult().estimatedPose.best;
        }
    }*/



    



    

}
