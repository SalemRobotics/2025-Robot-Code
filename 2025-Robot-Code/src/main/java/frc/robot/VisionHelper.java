package frc.robot;

<<<<<<< Updated upstream
=======
import edu.wpi.first.math.Matrix;
>>>>>>> Stashed changes
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionHelper {
    private Pose2d pose;
    private double time;
    private Matrix<N3, N1> visionMeasurementStdDevs;

    public VisionHelper(Pose2d pose, double time, Matrix<N3, N1> visionMeasurementStdDevs) {
        this.pose = pose;
        this.time = time;
        this.visionMeasurementStdDevs = visionMeasurementStdDevs;
    }

    public Pose2d getPose() {
        return pose;
    }

    public double getTime() {
        return time;
    }

    public Matrix<N3, N1> getVisionMeasurementStdDevs() {
        return visionMeasurementStdDevs;
    }
}
