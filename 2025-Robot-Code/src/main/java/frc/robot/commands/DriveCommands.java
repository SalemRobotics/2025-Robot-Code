package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveCommands {
    
    private static final double DEADBAND = 0.1;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;

    public static final double DRIVE_BASE_RADIUS = Math.max(
        Math.max(
            Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)
        ),
        Math.max(
            Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        )
    );

    private static double getMaxLinearSpeedMetersPerSecond() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /**
     * Gets a linear velocity as input from the driver joystick, treating the X and Y component as legs of a right triangle,
     * and calculates the hypotenuse 
     * @param x - the X component of the magnitude
     * @param y - the Y component of the magnitude
     * @return
     * 
     */
    private static Translation2d getLinearVelocityFromJoystick(double x, double y) {
        // Apply a deadband to the joystick inputs
        double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(x, y));

        // Squared inputs
        magnitude = magnitude * magnitude;

        return new Pose2d(new Translation2d(), linearDirection).transformBy(new Transform2d(magnitude, 0.0, new Rotation2d())).getTranslation();
    }
    
    /***
     * Robot relative drive command that uses the joystick for linear control towards a pose.
     * This uses a PID for aligning with the target laterally, and PID for angular control. Used for approaching
     * a known pose, from a short distance away. The approachSupplier must supply a Pose2d with a rotation facing AWAY from
     * the target.
     * @param drive
     * @param ySupplier
     * @param approachSupplier
     * @return
     */
    public static Command joystickApproach(CommandSwerveDrivetrain drive, DoubleSupplier ySupplier, Supplier<Pose2d> approachSupplier) {
        ProfiledPIDController angleController = new ProfiledPIDController(ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        ProfiledPIDController alignController = new ProfiledPIDController(1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        alignController.setGoal(0);

        return Commands.run(
        () -> { // Command
            Translation2d currentTranslation = drive.getState().Pose.getTranslation();
            Translation2d approachTranslation = approachSupplier.get().getTranslation();
            double distanceToApproach = currentTranslation.getDistance(approachTranslation);
            
            Rotation2d alignmentDirection = approachSupplier.get().getRotation();

            // Find lateral distance to Goal Pose
            Translation2d goalTranslation = new Translation2d(
                alignmentDirection.getCos() * distanceToApproach + approachTranslation.getX(),
                alignmentDirection.getSin() * distanceToApproach + approachTranslation.getY()
            );

            Translation2d robotToGoal = currentTranslation.minus(goalTranslation);
            double distanceToGoal = Math.hypot(robotToGoal.getX(), robotToGoal.getY());

            // Calculate lateral linear velocity
            Translation2d offsetVector = new Translation2d(alignController.calculate(distanceToGoal), 0).rotateBy(robotToGoal.getAngle());

            // Calculate total linear velocity
            Translation2d linearVelocity = getLinearVelocityFromJoystick(
                0, 
                ySupplier.getAsDouble()
            ).rotateBy(approachSupplier.get().getRotation()).rotateBy(Rotation2d.kCCW_90deg).plus(offsetVector);

            double theta = angleController.calculate(drive.getState().Pose.getRotation().getRadians(), approachSupplier.get().getRotation().rotateBy(Rotation2d.k180deg).getRadians());

            ChassisSpeeds speeds = new ChassisSpeeds(
                linearVelocity.getX() * getMaxLinearSpeedMetersPerSecond(), 
                linearVelocity.getY() * getMaxLinearSpeedMetersPerSecond(),
                theta
            );
            // Create a robot-centric swerve request to drive to the approach target, using the calculated chassis speeds.
            SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();
            drive.applyRequest(() -> request
                .withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getState().Pose.getRotation())));
        },
        drive // Requirements
        ).beforeStarting(() -> angleController.reset(drive.getState().Pose.getRotation().getRadians()));
    }
}
