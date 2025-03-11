package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionHelper {
    public Pose2d pose;
    public double time;

    public VisionHelper(Pose2d pose, double time){
        this.pose = pose;
        this.time = time;
    }

    public Pose2d getPose(){
        return pose;
    }

    public double getTime(){
        return time;
    }
}
