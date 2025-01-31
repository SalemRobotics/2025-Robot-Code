package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class EndEffector extends SubsystemBase {
    private DigitalInput mLineBreakerBack;
    private DigitalInput mLineBreakerFront;

    private final TalonFXConfiguration mConfig = new TalonFXConfiguration();
    private final TalonFX mEffectorMotorA = new TalonFX(0);
    //Could we reverse the break beam value to have it set to true if the object is in the way rather than the vice versa
    public Command jog(double speed){
        if (mLineBreakerFront.get() == true){
            while (mLineBreakerBack.get() == false){
                return runEnd(
                    () -> mEffectorMotorA.set(speed), 
                    () -> mEffectorMotorA.stopMotor());
            }
        }else if (mLineBreakerBack.get() == true){
            while (mLineBreakerFront.get() == false){
                return runEnd(
                    () -> mEffectorMotorA.set(-speed), 
                    () -> mEffectorMotorA.stopMotor());
            }
        }
                return null;
    } 
}
