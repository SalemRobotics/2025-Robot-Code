package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EndEffector extends SubsystemBase {
    private enum EndEffectorPhase {
        kNoCoral,
        kIntakingCoral,
        kAwaitingDelay,
        kRepositioningCoral,
        kInPosition,
    }

    private EndEffectorPhase mCurrentPhase = EndEffectorPhase.kNoCoral;
    // timer to represent the delay we use to stop the momentum of the coral
    private Timer mIntakeDelay = new Timer();
    private boolean mHasCoral = false;
    private double mEjectSpeed = EndEffectorConstants.kDefaultEjectSpeed;
    private final DigitalInput mEntranceLineBreaker = new DigitalInput(EndEffectorConstants.kEntranceBreakerPort);
    private final DigitalInput mExitLineBreaker = new DigitalInput(EndEffectorConstants.kExitBreakerPort);

    private final TalonFX mEffectorMotor = new TalonFX(EndEffectorConstants.kMotorPort, "rio");

    public EndEffector() {
        mEffectorMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Entrance", mEntranceLineBreaker.get());
        SmartDashboard.putBoolean("Exit", mExitLineBreaker.get());
        SmartDashboard.putString("End Effector State", mCurrentPhase.toString().substring(1));
        SmartDashboard.putBoolean("Awaiting Delay", mCurrentPhase == EndEffectorPhase.kAwaitingDelay);
    }

    // OLD IMPLEMENTATION - DO NOT DELETE
    // Could we reverse the break beam value to have it set to true if the object is
    // in the way rather than the vice versa

    public Command centerCoral() {
        return new RunCommand(() -> {
            if (!mExitLineBreaker.get() && !mEntranceLineBreaker.get()) {
                mHasCoral = true;
                mEffectorMotor.stopMotor();
            } else if (!mExitLineBreaker.get() && mEntranceLineBreaker.get()) {
                mEffectorMotor.set(-EndEffectorConstants.kAdjustSpeed);
            } else if (!mEntranceLineBreaker.get()) {
                mEffectorMotor.set(EndEffectorConstants.kAdjustSpeed);
            } else {
                if (mEntranceLineBreaker.get()) {
                    mHasCoral = false;
                }
                mEffectorMotor.set(EndEffectorConstants.kIdleSpeed);
                // mEffectorMotor.setControl(new VelocityVoltage(20.0));
            }
        }, this);
    }

    // getter for mHasCoral
    public boolean hasCoral() {
        return mHasCoral;
    }

    // setter for mEjectSpeed
    public void setEjectSpeed(double speed) {
        mEjectSpeed = speed;
    }

    // checker for controller rumbling
    public boolean isInPosition() {
        return hasCoral() && mCurrentPhase == EndEffectorPhase.kInPosition;
    }

    public Command ejectCoral() {
        return Commands.run(() -> mEffectorMotor.set(mEjectSpeed), this);
    }
}
