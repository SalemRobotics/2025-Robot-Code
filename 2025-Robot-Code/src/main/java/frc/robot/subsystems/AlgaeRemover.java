package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeRemover extends SubsystemBase {
    private final TalonFXConfiguration mConfig = new TalonFXConfiguration();
    private final TalonFX mAlgaeMotor = new TalonFX(AlgaeConstants.kAlgaeMotorPort, AlgaeConstants.kAlgaeMotorBus);
    private final PositionVoltage mPositionVoltage;

    public AlgaeRemover() {
        mAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);
        mPositionVoltage = new PositionVoltage(0).withSlot(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Algae Remover Extended",
                mPositionVoltage.Position == AlgaeConstants.kAlgaeExtendedRotation);
    }

    /*
     * Pivots algae arm into position to remove algae
     */
    public Command pivotAlgaeArm() {
        return runEnd(
                () -> mAlgaeMotor.setControl(mPositionVoltage.withPosition(AlgaeConstants.kAlgaeExtendedRotation)),
                () -> mAlgaeMotor.setControl(mPositionVoltage.withPosition(AlgaeConstants.kAlgaeStowedRotation)));
    }

}
