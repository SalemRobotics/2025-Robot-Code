package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix6.hardware.TalonFX;

public class EndEffector extends SubsystemBase {
    private DigitalInput mLineBreakerBack;
    private DigitalInput mLineBreakerFront;
    private boolean mHasCoral = false;

    private final TalonFX mEffectorMotorA = new TalonFX(0, "rio");

    // Could we reverse the break beam value to have it set to true if the object is
    // in the way rather than the vice versa
    public Command centerCoral(EndEffector endEffector) {
        return new RunCommand(() -> {
            if (mLineBreakerFront.get() && mLineBreakerBack.get()) {
                mEffectorMotorA.stopMotor();
                mHasCoral = true;
            } else if (mLineBreakerFront.get()) {
                mEffectorMotorA.set(EndEffectorConstants.kEndEffectorMotorSpeed);
            } else if (mLineBreakerBack.get()) {
                mEffectorMotorA.set(-EndEffectorConstants.kEndEffectorMotorSpeed);
            } else {
                mHasCoral = false;
                mEffectorMotorA.set(EndEffectorConstants.kEndEffectorMotorSpeed);
            }
        }, endEffector);
        
    }
    // TODO: make getter for mHasCoral
    
}
