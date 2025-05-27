package frc.robot.selfdriving;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.commands.SelfDriveCommands;
import frc.robot.selfdriving.Objective.Intaking;
import frc.robot.selfdriving.Objective.Scoring;
import frc.robot.selfdriving.SelfDriveTarget.AllianceSide;
import frc.robot.selfdriving.SelfDriveTarget.CoralStation.IntakePosition;
import frc.robot.subsystems.AlgaeRemover;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class SelfDriving extends SubsystemBase {
    private IntakePosition preferredCoralStationSide = IntakePosition.Middle;
    
    public Command nearestCoralStationIntake(Drivetrain drive, EndEffector endEffector, AlgaeRemover remover, Elevator elevator, CommandXboxController controller) {
        return Commands.defer(
            () -> SelfDriveCommands.selfDrivingIntake(
                Intaking.nearestCoralStationIntake(drive.getPose(), preferredCoralStationSide), 
                drive, 
                endEffector, 
                remover, 
                elevator, 
                false, 
                controller::getLeftX, controller::getLeftY,
                controller::getRightX), 
                Set.of(drive, endEffector, remover, elevator, this));
    }
    public Command nearestCoralScore(Drivetrain drive, EndEffector endEffector, AlgaeRemover remover, Elevator elevator, CommandXboxController controller) {
        return Commands.defer(
            () -> {
                final Scoring scoring = Scoring.nearestCoralPole(drive.getPose(), 4);

                return SelfDriveCommands.selfDrivingScore(
                    scoring, 
                    drive, 
                    endEffector, 
                    remover, 
                    elevator, 
                    false, 
                    Quadrant.fromPose(scoring.getScoringPose()).middle(), 
                    preferredCoralStationSide, 
                    AllianceSide.Left, 
                    controller::getLeftX, controller::getLeftY, 
                    controller::getRightX);
            },
            Set.of(drive, endEffector, remover, elevator, this)
        );
    }
}
