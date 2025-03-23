package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeRemover extends SubsystemBase {
    private final TalonFX mAlgaeMotor = new TalonFX(AlgaeConstants.kAlgaeMotorPort, "rio");
    private final Timer mDeployTimer = new Timer();

    public AlgaeRemover() {
        mAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.withStatorCurrentLimit(10);

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = mAlgaeMotor.getConfigurator().apply(config);
            if (status.isOK())
                break;
        }
        if (!status.isOK())
            System.err.println("Failed to configure algea remover motor: " + status.toString());

        mAlgaeMotor.setPosition(0);
    }

    /*
     * Pivots algae arm into position to remove algae
     * CURRENTLY NOT FUNCTIONAL - (physical) change belt to a chain
     */
    public Command pivotAlgaeArm(BooleanSupplier goDown) {
        return Commands.waitSeconds(0.05)
                .beforeStarting(() -> mAlgaeMotor.set(goDown.getAsBoolean() ? 0.5 : -0.5))
                .andThen(
                        run(() -> {
                            if (MathUtil.isNear(0, mAlgaeMotor.getVelocity().getValueAsDouble(), 0.1))
                                mDeployTimer.start();
                        }).until(() -> mDeployTimer.hasElapsed(0.25)))
                .finallyDo(() -> {
                    mAlgaeMotor.stopMotor();
                    mDeployTimer.reset();
                });
    }
}