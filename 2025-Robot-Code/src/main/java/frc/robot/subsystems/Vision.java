package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class Vision {
    final AprilTagFieldLayout mFieldLayout;
    final PhotonCamera mCamera1;
    final PhotonCamera mCamera2;

    Vision() {
        mCamera1 = new PhotonCamera("");
        mCamera2 = new PhotonCamera("");
        mFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }

    

}
