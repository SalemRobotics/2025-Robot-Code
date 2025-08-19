package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  private Setpoint lastSetpoint = Setpoint.Stowed, currentSetpoint = Setpoint.Stowed;

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

  private void setRotationalTarget(Angle rots) {
    io.setTarget(rots, lastSetpoint.isElevated() && currentSetpoint.isElevated());
  }

  private void newSetpoint(Setpoint setpoint) {
    lastSetpoint = currentSetpoint;
    currentSetpoint = setpoint;
  }

  public Command setTarget(Setpoint setpoint) {
    return runOnce(() -> setRotationalTarget(setpoint.target))
        .beforeStarting(() -> newSetpoint(setpoint));
  }

  public Command stow() {
    return runOnce(() -> setRotationalTarget(Setpoint.Stowed.target))
        .beforeStarting(() -> newSetpoint(Setpoint.Stowed));
  }

  public boolean madeProgress(double percentage) {
    return inputs.leaderPosition.gt(currentSetpoint.target.times(percentage));
  }

  public boolean shouldEjectFast() {
    return currentSetpoint == Setpoint.L4;
  }
}
