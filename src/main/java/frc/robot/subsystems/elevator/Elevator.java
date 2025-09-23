package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  public static enum Setpoint {
    Stowed(0.25),
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
  private final Map<Setpoint, Command> cachedCommands = new HashMap<>(Setpoint.values().length);

  public final Trigger isAtSetpoint =
      new Trigger(() -> inputs.leaderPosition.isNear(currentSetpoint.target, Rotations.of(0.05)));

  public Elevator(ElevatorIO motorIO) {
    io = motorIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator/Motors", inputs);

    SmartDashboard.putNumber("Elevator Setpoint", currentSetpoint.target.in(Rotations));
  }

  public Command setTarget(Setpoint setpoint) {
    return cachedCommands.computeIfAbsent(setpoint, s -> runOnce(
      () -> io.setTarget(setpoint)
    ).beforeStarting(() -> currentSetpoint = s));
  }

  public Command stow() {
    return setTarget(Setpoint.Stowed);
  }

  public boolean madeProgress(double percentage) {
    return inputs.leaderPosition.gt(currentSetpoint.target.times(percentage));
  }

  public boolean shouldEjectFast() {
    return currentSetpoint == Setpoint.L4;
  }
}
