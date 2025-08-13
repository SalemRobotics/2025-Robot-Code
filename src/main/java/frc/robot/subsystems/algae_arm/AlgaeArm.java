package frc.robot.subsystems.algae_arm;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.algae_arm.AlgaeArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.io.talon.TalonFXIO;
import frc.robot.util.io.talon.TalonFXIOImpl;
import frc.robot.util.io.talon.TalonFXIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class AlgaeArm extends SubsystemBase {
  private final TalonFXIO io;
  private final TalonFXIOInputsAutoLogged inputs = new TalonFXIOInputsAutoLogged();

  private final Trigger hasDeployed =
      new Trigger(
          () -> MathUtil.isNear(inputs.position.in(Rotations), kDeployedAngle.in(Rotations), 0.01));

  private final Trigger hasStowed =
      new Trigger(
          () -> MathUtil.isNear(inputs.position.in(Rotations), kDeployedAngle.in(Rotations), 0.01));

  public AlgaeArm(TalonFXIO motorIO) {
    io = motorIO;

    setDefaultCommand(stow());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("AlgaeArm/Motor", inputs);
  }

  public static AlgaeArm createReal() {
    return new AlgaeArm(new TalonFXIOImpl(kMotorID, kBus, kAlgaeArmCallback));
  }

  public static AlgaeArm createSim() {
    return new AlgaeArm(new AlgaeArmIOSim());
  }

  public Command deploy() {
    return Commands.sequence(
        runOnce(() -> io.setDutyCycle(.8)),
        Commands.waitUntil(hasDeployed),
        runOnce(() -> io.stopMotor()));
  }

  public Command stow() {
    return Commands.sequence(
        runOnce(() -> io.setDutyCycle(-.8)),
        Commands.waitUntil(hasStowed),
        runOnce(() -> io.stopMotor()));
  }
}
