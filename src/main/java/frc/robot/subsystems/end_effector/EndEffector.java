package frc.robot.subsystems.end_effector;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.subsystems.end_effector.EndEffectorConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.io.talon.TalonFXIO;
import frc.robot.util.io.talon.TalonFXIOImpl;
import frc.robot.util.io.talon.TalonFXIOInputsAutoLogged;
import frc.robot.util.io.talon.TalonFXIOSimImpl;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private final TalonFXIO motorIO;
  private final TalonFXIOInputsAutoLogged motorInputs = new TalonFXIOInputsAutoLogged();
  private final BeamBreakIO entranceIO, exitIO;
  private final BeamBreakIOInputsAutoLogged entranceInputs = new BeamBreakIOInputsAutoLogged(),
      exitInputs = new BeamBreakIOInputsAutoLogged();
  private boolean coralInPosition, firstTime;
  private final boolean jogDuringIntake;

  public EndEffector(TalonFXIO motor, BeamBreakIO entrance, BeamBreakIO exit, boolean jog) {
    motorIO = motor;
    entranceIO = entrance;
    exitIO = exit;

    setDefaultCommand(teleIntake());
    jogDuringIntake = jog;
    SmartDashboard.putBoolean("Demo Intake", false);
  }

  public static EndEffector createReal() {
    CurrentLimitsConfigs configs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(70))
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(120))
            .withSupplyCurrentLimitEnable(true);

    return new EndEffector(
        new TalonFXIOImpl(
            kMotorID, kMotorBus, new TalonFXConfiguration().withCurrentLimits(configs)),
        new BeamBreakIODIO(kEntranceBreakerID),
        new BeamBreakIODIO(kExitBreakerID),
        true);
  }

  public static EndEffector createSim() {
    return new EndEffector(
        new TalonFXIOSimImpl(kMotorID, kMotorBus, true, kSystemMOI, 1),
        new BeamBreakIOSupplierSim(() -> false),
        new BeamBreakIOSupplierSim(() -> false),
        false);
  }

  private final void jogCoral() {
    final boolean entrance = entranceInputs.isBroken;
    final boolean exit = exitInputs.isBroken;

    if (jogDuringIntake) {
      if (!entrance && !exit) {
        motorIO.setDutyCycle(kIdleSpeed);

        coralInPosition = false;
        firstTime = true;
      } else if (entrance && exit) {
        if (firstTime) {
          motorIO.setDutyCycle(kIntakeSpeed / 2);
          coralInPosition = false;
        } else {
          motorIO.stopMotor();
          coralInPosition = true;
        }
      } else if (entrance && !exit) {
        motorIO.setDutyCycle(kIntakeSpeed);

        coralInPosition = false;
      } else if (!entrance && exit) {
        motorIO.setDutyCycle(-kDriveBackSpeed);

        coralInPosition = false;
        firstTime = false;
      }
    } else {
      if (entrance && exit) {
        motorIO.stopMotor();
      } else {
        motorIO.setDutyCycle(kIdleSpeed);
      }
    }
  }

  @Override
  public void periodic() {
    motorIO.updateInputs(motorInputs);
    entranceIO.updateInputs(entranceInputs);
    exitIO.updateInputs(exitInputs);

    Logger.processInputs("EndEffector/Motor", motorInputs);
    Logger.processInputs("EndEffector/EntranceSensor", entranceInputs);
    Logger.processInputs("EndEffector/ExitSensor", exitInputs);

    SmartDashboard.putBoolean("Coral In Position", coralInPosition);
    SmartDashboard.putBoolean("First Time", firstTime);
    SmartDashboard.putBoolean("Entrance Detected", entranceInputs.isBroken);
    SmartDashboard.putBoolean("Exit Detected", exitInputs.isBroken);
  }

  public void resetState() {
    motorIO.stopMotor();
    coralInPosition = false;
    firstTime = true;
  }

  public void onEnable() {
    if (entranceIO.isBroken() && exitIO.isBroken()) {
      firstTime = false;
      coralInPosition = true;
    } else resetState();
  }

  public Command teleIntake() {
    return runOnce(this::jogCoral).andThen(Commands.waitSeconds(0.02));
  }

  public Command autoIntake() {
    return run(this::jogCoral).finallyDo(this::resetState);
  }

  public Command teleScoreCoral(BooleanSupplier ejectFast) {
    return run(
        () -> {
          motorIO.setDutyCycle(ejectFast.getAsBoolean() ? kFastEjectSpeed : kSlowEjectSpeed);

          firstTime = true;
          coralInPosition = false;
        });
  }

  public Command autoScoreCoral(BooleanSupplier elevatorIsAtHeight) {
    return Commands.sequence(
        // Commands.waitUntil(elevatorIsAtHeight),
        // Commands.print("Elevator is at height"),
        Commands.waitSeconds(0.25),
        runOnce(() -> motorIO.setDutyCycle(kAutoEjectSpeed)),
        Commands.print("Motor is set to " + kAutoEjectSpeed),
        Commands.race(Commands.waitSeconds(0.2), Commands.waitUntil(() -> !exitIO.isBroken())),
        Commands.print("Coral out"),
        runOnce(this::resetState));
  }

  public Command scoreBarge() {
    return runOnce(() -> motorIO.setDutyCycle(-kAlgaeBargeSpeed)).beforeStarting(this::resetState);
  }

  public Command scoreProcessor() {
    return runOnce(() -> motorIO.setDutyCycle(-kAlgaeProcessorSpeed))
        .beforeStarting(this::resetState);
  }

  public Command intakeAlgae() {
    return run(() -> motorIO.setDutyCycle(kIdleSpeed)).beforeStarting(this::resetState);
  }
}
