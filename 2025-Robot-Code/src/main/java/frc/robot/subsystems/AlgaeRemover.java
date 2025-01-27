package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeRemover extends SubsystemBase{
    private final TalonFXConfiguration mConfig = new TalonFXConfiguration();
    private final TalonFX mAlgaeMotor = new TalonFX(AlgaeConstants.kAlgaeMotorPort, AlgaeConstants.kAlgaeMotorBus);

    public AlgaeRemover(){
        mAlgaeMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public Command runAlgaeRemover(double speed){
        return runEnd(
            () -> mAlgaeMotor.set(speed), 
            () -> mAlgaeMotor.stopMotor());
    }


}
