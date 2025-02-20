package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class EndEffector extends SubsystemBase {
    private final DigitalInput mEntranceLineBreaker = new DigitalInput(EndEffectorConstants.kEntranceLineBreakerPort);
    private final DigitalInput mExitLineBreaker = new DigitalInput(EndEffectorConstants.kExitLineBreakerPort);
    private boolean mHasCoral = false;

    private final TalonFXConfiguration mConfig = new TalonFXConfiguration();
    private final TalonFX mEffectorMotor = new TalonFX(0, "rio0");
    private final TalonFX mAlgaeRemoverMotor = new TalonFX(1, "rio0");

    public EndEffector() {  
        FeedbackConfigs fbcfg = mConfig.Feedback;
        fbcfg.SensorToMechanismRatio = EndEffectorConstants.kSensorToMechanismRatio;

        Slot0Configs slot0 = mConfig.Slot0;
        slot0.kS = EndEffectorConstants.kAlgaeRemoverS;
        slot0.kV = EndEffectorConstants.kAlgaeRemoverV;
        slot0.kA = EndEffectorConstants.kAlgaeRemoverA;
        slot0.kP = EndEffectorConstants.kAlgaeRemoverP;
        slot0.kI = EndEffectorConstants.kAlgaeRemoverI;
        slot0.kD = EndEffectorConstants.kAlgaeRemoverD;
        slot0.kG = EndEffectorConstants.kAlgaeRemoverG;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = mAlgaeRemoverMotor.getConfigurator().apply(mConfig);

            if (status.isOK())
                break;
        }

        if (status.isError())
            DataLogManager.log("Failed to configure algae remover motor: " + status.toString());

        mAlgaeRemoverMotor.setPosition(0);
    }

    // Could we reverse the break beam value to have it set to true if the object is
    // in the way rather than the vice versa
    public Command centerCoral() {
        return new RunCommand(() -> {
            if (mExitLineBreaker.get() && !mEntranceLineBreaker.get()) {
                mHasCoral = true;
                mEffectorMotor.stopMotor();
            } else if (mExitLineBreaker.get()) {
                mEffectorMotor.set(EndEffectorConstants.kEndEffectorSlowSpeed);
            } else {
                if (!mEntranceLineBreaker.get()) {
                    mHasCoral = false;
                }
                mEffectorMotor.set(EndEffectorConstants.kEndEffectorFastSpeed);
            }
        }, this);

    }

    public boolean mHasCoralGetter() {
        return mHasCoral;
    }

    public Command ejectCoral() {
        return Commands.run(() -> mEffectorMotor.set(EndEffectorConstants.kEndEffectorFastSpeed));
    }

}
