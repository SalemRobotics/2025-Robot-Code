package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class MobilityAuto extends ParallelCommandGroup{
    CommandSwerveDrivetrain mDrivetrain;

    public MobilityAuto(CommandSwerveDrivetrain drivetrain){
        mDrivetrain = drivetrain;

        try{
            boolean isBlue = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
            
            List<Waypoint> waypoints = 
            PathPlannerPath.waypointsFromPoses(
                new Pose2d(
                    mDrivetrain.getState().Pose.getX(), 
                    mDrivetrain.getState().Pose.getY(), 
                    isBlue?
                    Rotation2d.fromDegrees(180): 
                    Rotation2d.fromDegrees(0)
                ),
                isBlue? new Pose2d(mDrivetrain.getState().Pose.getX() - 2.0, mDrivetrain.getState().Pose.getY(), Rotation2d.fromDegrees(180)):
                new Pose2d(mDrivetrain.getState().Pose.getX() + 2.0, mDrivetrain.getState().Pose.getY(), Rotation2d.fromDegrees(0))
            );

            PathConstraints constraints = DriveConstants.kMobilityConstraints;

            PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0.0, isBlue? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0))
            );
            addCommands(AutoBuilder.followPath(path));
        }catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            addCommands(Commands.none());
        }
        


    }

}
