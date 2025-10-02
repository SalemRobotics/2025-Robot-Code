package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Climber", inputs);
  }

  public Command deploy() {
    return runOnce(io::deploy)
        .andThen(Commands.idle(this))
        .finallyDo(io::stop)
        .withName("Deploy Climber");
  }

  public Command retract() {
    return runOnce(io::retract)
        .andThen(Commands.idle(this))
        .finallyDo(io::stop)
        .withName("Retract Climber");
  }
}
