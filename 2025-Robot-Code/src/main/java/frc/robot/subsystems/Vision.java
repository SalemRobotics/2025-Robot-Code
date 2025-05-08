package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.VisionHelper;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private final AprilTagFieldLayout m_fieldLayout;
    private final double linearStdDevBaseline = 0.02;
    private final double angularStdDevBaseline = 0.06;
    private final PhotonCamera mCamera1 = new PhotonCamera(VisionConstants.kCamera1Name);
    private final PhotonCamera mCamera2 = new PhotonCamera(VisionConstants.kCamera2Name);

    public static record PoseObservation(double timestamp, Pose3d pose, double ambiguity, int tagCount,
            double averageTagDistance) {
    }

    public Vision() {
        AprilTagFieldLayout layout;
        boolean customField = true;

        try {
            layout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().getAbsolutePath() + "/weldedlayout.json");
            customField = true;
        } catch (Exception e) {
            System.err.println("Error configuring an apriltag layout: " + e.getMessage() + "\n" + e.getStackTrace());
            layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        }

        m_fieldLayout = layout;
        SmartDashboard.putBoolean("Custom field: ", customField);
    }

    public ArrayList<VisionHelper> getVisionResults() {
        ArrayList<VisionHelper> outputs = new ArrayList<>();
        for (var observation : getObservations(mCamera1, VisionConstants.kRobotToCam1)) {
            double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
            double linearStdDev = linearStdDevBaseline * stdDevFactor;
            double angularStdDev = angularStdDevBaseline * stdDevFactor;
            outputs.add(new VisionHelper(
                    observation.pose.toPose2d(), observation.timestamp(),
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)));
        }
        for (var observation : getObservations(mCamera2, VisionConstants.kRobotToCam2)) {
            double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
            double linearStdDev = linearStdDevBaseline * stdDevFactor;
            double angularStdDev = angularStdDevBaseline * stdDevFactor;
            outputs.add(new VisionHelper(
                    observation.pose.toPose2d(), observation.timestamp(),
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)));
        }
        return outputs;
    }

    public List<PoseObservation> getObservations(PhotonCamera camera, Transform3d robotToCamera) {
        // Read new camera observations
        Set<Short> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
        for (var result : camera.getAllUnreadResults()) {
            // Add pose observation
            if (result.multitagResult.isPresent()) { // Multitag result
                var multitagResult = result.multitagResult.get();

                // Calculate robot pose
                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                // Calculate average tag distance
                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                    totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                }

                // Add tag IDs
                tagIds.addAll(multitagResult.fiducialIDsUsed);

                // Add observation
                poseObservations.add(
                        new PoseObservation(
                                result.getTimestampSeconds(), // Timestamp
                                robotPose, // 3D pose estimate
                                multitagResult.estimatedPose.ambiguity, // Ambiguity
                                multitagResult.fiducialIDsUsed.size(), // Tag count
                                totalTagDistance / result.targets.size() // Average tag distance
                        ));

            } else if (!result.targets.isEmpty()) { // Single tag result
                var target = result.targets.get(0);

                // Calculate robot pose
                var tagPose = m_fieldLayout.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget = new Transform3d(tagPose.get().getTranslation(),
                            tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    // Add tag ID
                    tagIds.add((short) target.fiducialId);

                    // Add observation
                    poseObservations.add(
                            new PoseObservation(
                                    result.getTimestampSeconds(), // Timestamp
                                    robotPose, // 3D pose estimate
                                    target.poseAmbiguity, // Ambiguity
                                    1, // Tag count
                                    cameraToTarget.getTranslation().getNorm() // Average tag distance
                            ));
                }
            }
        }

        // Save pose observations to inputs object
        return poseObservations;
    }
}