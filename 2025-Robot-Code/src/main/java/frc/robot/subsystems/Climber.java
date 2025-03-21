package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final Servo mServo = new Servo(0);
    private final TalonFX mClimberMotor = new TalonFX(30, "canivore0");

    public Climber() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        config.CurrentLimits.withStatorCurrentLimit(40.00);
        config.withCurrentLimits(config.CurrentLimits);

        for (int i = 0; i < 5; i++) {
            status = mClimberMotor.getConfigurator().apply(config);

            if (status.isOK())
                break;
        }

        if (status.isError())
            System.err.println("Failed to configure elevator motors: " + status.toString());
    }

    public Command climb() {
        return runOnce(() -> {
            mClimberMotor.set(.75);
        });
    }

    public Command prepClimb() {
        return run(() -> {
            mServo.setAngle(120);
        });
    }

    public Command declimb() {
        return runOnce(() -> {
            mClimberMotor.set(-.75);
        });
    }
    public Command stopMotor() {
        return runOnce(() -> {
            mClimberMotor.stopMotor();
            mServo.setAngle(75);
        });
        
    }
}
