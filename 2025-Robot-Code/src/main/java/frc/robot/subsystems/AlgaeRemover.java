package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeRemover extends SubsystemBase {
    private final TalonFXConfiguration mConfig = new TalonFXConfiguration();
    private final TalonFX mAlgaeMotor = new TalonFX(AlgaeConstants.kAlgaeMotorPort, "rio");
    private final PositionVoltage mPositionVoltage = new PositionVoltage(0).withSlot(0);

    public AlgaeRemover() {
        mAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);
        mConfig.Feedback.SensorToMechanismRatio = AlgaeConstants.kSensorToMechanismRatio;
        mAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);

        Slot0Configs slot0 = mConfig.Slot0;
        slot0.kP = 8.0;
        slot0.kI = 0;
        slot0.kD = 0.1;

        mConfig.Voltage.withPeakForwardVoltage(Volts.of(11)).withPeakReverseVoltage(Volts.of(-11));

        Slot1Configs slot1 = mConfig.Slot1;
        slot1.kP = 60;
        slot1.kI = 0;
        slot1.kD = 6;

        mConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(120)).withPeakReverseTorqueCurrent(Amps.of(-120));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = mAlgaeMotor.getConfigurator().apply(mConfig);
            if (status.isOK())
                break;
        }
        if (!status.isOK())
            DataLogManager.log("Failed to configure algea remover motor: " + status.toString());

        mAlgaeMotor.setPosition(0);
    }

    /*
     * Pivots algae arm into position to remove algae
     */
    public Command pivotAlgaeArm(double position) {
        return Commands.run(() -> mAlgaeMotor.setControl(mPositionVoltage.withPosition(position)), this);
    }
}
