package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Servo;

public class ServoIOReal implements ServoIO {
  private final Servo servo;

  public ServoIOReal(int channel) {
    servo = new Servo(channel);
  }

  @Override
  public void updateInputs(ServoIOInputs inputs) {
    inputs.angle = servo.getAngle();
  }

  @Override
  public void setAngle(double angle) {
    servo.setAngle(angle);
  }
}
