package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.io.talon.TalonFXIO;
import frc.robot.util.io.talon.TalonFXIOImpl;
import frc.robot.util.io.talon.TalonFXIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final TalonFXIO motor;
  private final TalonFXIOInputsAutoLogged motorInputs = new TalonFXIOInputsAutoLogged();

  private final ServoIO servo;
  private final ServoIOInputsAutoLogged servoInputs = new ServoIOInputsAutoLogged();

  public Climber(TalonFXIO motorIO, ServoIO servoIO) {
    motor = motorIO;
    servo = servoIO;
  }

  @Override
  public void periodic() {
    motor.updateInputs(motorInputs);
    servo.updateInputs(servoInputs);

    Logger.processInputs("Climber/Motor", motorInputs);
    Logger.processInputs("Climber/Servo", servoInputs);
  }

  public static Climber createReal() {
    return new Climber(new TalonFXIOImpl(kMotorId, Constants.kCanivore), new ServoIOReal(kServoId));
  }

  public Command deploy() {
    return runOnce(
        () -> {
          motor.setDutyCycle(-0.9);
          servo.setAngle(80);
        });
  }

  public Command retract() {
    return runOnce(
        () -> {
          motor.setDutyCycle(0.9);
        });
  }

  public Command stop() {
    return runOnce(
        () -> {
          motor.stopMotor();
          servo.setAngle(50);
        });
  }
}
