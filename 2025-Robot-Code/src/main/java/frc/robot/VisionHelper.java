package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionHelper {
    private Optional<Pose2d> pose;
    private double time;

    public VisionHelper(Optional<Pose2d> pose, double time) {
        this.pose = pose;
        this.time = time;
    }

    public Optional<Pose2d> getPose() {
        return pose;
    }

    public double getTime() {
        return time;
    }
}
