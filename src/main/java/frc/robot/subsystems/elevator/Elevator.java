package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  public static enum Setpoint {
    Stowed(0),
    L2(1.67),
    L3(2.82),
    L4(4.7),
    AlgaeLow(0.32),
    AlgaeHigh(1.625),
    Barge(4.7);

    final Angle target;

    Setpoint(double rots) {
      target = Rotations.of(rots);
    }

    public boolean isElevated() {
      return this != Stowed;
    }

    public boolean isCoral() {
      switch (this) {
        case L2:
        case L3:
        case L4:
          return true;

        default:
          return false;
      }
    }

    public boolean isAlgae() {
      switch (this) {
        case AlgaeLow:
        case AlgaeHigh:
        case Barge:
          return true;

        default:
          return false;
      }
    }
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private Setpoint currentSetpoint = Setpoint.Stowed;

  public final Trigger isAtSetpoint =
      new Trigger(() -> inputs.leaderPosition.isNear(currentSetpoint.target, Rotations.of(0.1)));

  public Elevator(ElevatorIO motorIO) {
    io = motorIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator/Motors", inputs);

    Logger.recordOutput("Elevator/Setpoint", currentSetpoint.target.in(Rotations));
    Logger.recordOutput("Elevator/IsAtHeight", isAtSetpoint.getAsBoolean());
  }

  public Command setTarget(Setpoint setpoint) {
    return Commands.sequence(
        runOnce(
            () -> {
              currentSetpoint = setpoint;
              io.setTarget(setpoint);
            }),
        Commands.idle(this));
  }

  public Command stow() {
    return setTarget(Setpoint.Stowed);
  }

  public boolean madeProgress(double percentage) {
    return inputs.leaderPosition.gte(currentSetpoint.target.times(percentage));
  }

  public boolean shouldEjectFast() {
    return currentSetpoint == Setpoint.L4;
  }
}
