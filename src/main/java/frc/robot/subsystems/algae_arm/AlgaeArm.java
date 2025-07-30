package frc.robot.subsystems.algae_arm;

import static frc.robot.subsystems.algae_arm.AlgaeArmConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.io.talon.TalonFXIO;
import frc.robot.util.io.talon.TalonFXIOImpl;

public class AlgaeArm extends SubsystemBase {
  private final TalonFXIO io;

  public AlgaeArm(TalonFXIO motorIO) {
    io = motorIO;

    setDefaultCommand(stow());
  }

  public static AlgaeArm createReal() {
    return new AlgaeArm(new TalonFXIOImpl(kMotorID, kBus, kAlgaeArmCallback));
  }

  public static AlgaeArm createSim() {
    return new AlgaeArm(new AlgaeArmIOSim());
  }

  public Command deploy() {
    return runOnce(() -> io.setPosition(kDeployedAngle));
  }

  public Command stow() {
    return runOnce(() -> io.setPosition(kStowedAngle));
  }
}
