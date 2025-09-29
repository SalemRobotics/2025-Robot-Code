package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.algae_arm.AlgaeArm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.Setpoint;
import frc.robot.subsystems.end_effector.EndEffector;

public final class Superstructure {
  private final EndEffector endEffector;
  private final Elevator elevator;
  private final AlgaeArm algaeArm;

  public Superstructure(EndEffector endEffector, Elevator elevator, AlgaeArm algaeArm) {
    this.endEffector = endEffector;
    this.elevator = elevator;
    this.algaeArm = algaeArm;
  }

  public Command bargeShot() {
    return Commands.parallel(
        elevator.setTarget(Setpoint.Barge),
        Commands.waitSeconds(0.35).andThen(endEffector.scoreBarge(), algaeArm.stow()));
  }

  public Command scoreCoral(boolean inAuto) {
    if (inAuto) {
      return Commands.print("Scoring coral")
          .andThen(endEffector.autoScoreCoral(elevator.isAtSetpoint));
    } else {
      return endEffector.teleScoreCoral(elevator::shouldEjectFast);
    }
  }

  public Command algaeMode() {
    return endEffector.intakeAlgae().alongWith(algaeArm.deploy());
  }
}
