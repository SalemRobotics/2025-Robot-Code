package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeRemover extends SubsystemBase {
    private final TalonFX mAlgaeMotor = new TalonFX(AlgaeConstants.kAlgaeMotorPort, "rio");

    public AlgaeRemover() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.withSensorToMechanismRatio(25);
        config.CurrentLimits.withStatorCurrentLimit(10);

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = mAlgaeMotor.getConfigurator().apply(config);
            if (status.isOK())
                break;
        }
        if (!status.isOK())
            System.err.println("Failed to configure algae remover motor: " + status.toString());
        
        mAlgaeMotor.setPosition(0);
        mAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public final Trigger hasDeployedTrigger = new Trigger(
        () -> MathUtil.isNear(.283, mAlgaeMotor.getPosition().getValueAsDouble(), .01)
    );

    public final Trigger hasStowedTrigger = new Trigger( 
        () -> MathUtil.isNear(0.0, mAlgaeMotor.getPosition().getValueAsDouble(), .01)
    );

    public Command deployArm() {
        return Commands.sequence(
            runOnce(() -> mAlgaeMotor.set(.75)),
            Commands.race(
                Commands.waitUntil(hasDeployedTrigger),
                Commands.waitSeconds(.5)
            ),
            runOnce(() -> mAlgaeMotor.stopMotor())
        );
    }

    public Command stowArm() {
        return Commands.sequence(
            runOnce(() -> mAlgaeMotor.set(-.75)),
            Commands.race(
                Commands.waitUntil(hasStowedTrigger),
                Commands.waitSeconds(.5)
            ),
            runOnce(() -> mAlgaeMotor.stopMotor())
        );
    }
    public Command stowArm(BooleanSupplier currentlyDeployed) {
        return stowArm().unless(currentlyDeployed);
    }
    public Command dropArm() {
        return Commands.sequence(
            runOnce(() -> {
                removeDefaultCommand();
                mAlgaeMotor.setNeutralMode(NeutralModeValue.Coast);
                mAlgaeMotor.set(0.5);
            }),
            Commands.waitSeconds(1),
            runOnce(() -> {
                mAlgaeMotor.stopMotor();
            })
        );
    }
}