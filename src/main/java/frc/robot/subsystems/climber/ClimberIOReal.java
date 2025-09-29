package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.util.phoenix.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Servo;

public class ClimberIOReal implements ClimberIO {
    private final Servo servo = new Servo(SERVO_ID);
    private final TalonFX talon = new TalonFX(MOTOR_ID, MOTOR_BUS);
    private final StatusSignal<AngularVelocity> velocity = talon.getVelocity();

    public ClimberIOReal() {
        tryUntilOk(5, () -> talon.getConfigurator().apply(MOTOR_CONFIG));
        velocity.setUpdateFrequency(100);
        talon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        velocity.refresh();

        inputs.connected = velocity.getStatus().isOK();
        inputs.motorSpeed = velocity.getValue();
        inputs.servoSetpoint = Degrees.of(servo.getAngle());
    }

    @Override
    public void deploy() {
        servo.setAngle(80);
        talon.set(-1);
    }

    @Override
    public void retract() {
        talon.set(1);
    }

    @Override
    public void stop() {
        talon.stopMotor();
        servo.setAngle(50);
    }
}
