package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase{
    final AprilTagFieldLayout mFieldLayout;
    final PhotonCamera mCamera1;
    final PhotonCamera mCamera2;

    Vision() {
        mCamera1 = new PhotonCamera(VisionConstants.kCamera1Name);
        mCamera2 = new PhotonCamera(VisionConstants.kCamera1Name);
        mFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    }



    

}
