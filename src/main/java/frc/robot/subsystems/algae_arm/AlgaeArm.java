package frc.robot.subsystems.algae_arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class AlgaeArm extends SubsystemBase {
  private final AlgaeArmIO io;
  private final AlgaeArmIOInputsAutoLogged inputs = new AlgaeArmIOInputsAutoLogged();

  public AlgaeArm(AlgaeArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("AlgaeArm", inputs);
  }

  public Command deploy() {
    return runOnce(io::deploy).andThen(Commands.idle(this)).withName("Deploy Algae Arm");
  }

  public Command stow() {
    return runOnce(io::stow).andThen(Commands.idle(this)).withName("Stow Algae Arm");
  }
}
