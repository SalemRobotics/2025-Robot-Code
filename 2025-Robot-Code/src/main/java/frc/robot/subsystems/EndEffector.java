package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.EndEffectorConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EndEffector extends SubsystemBase {
    private final DigitalInput mEntranceLineBreaker = new DigitalInput(EndEffectorConstants.kEntranceBreakerPort);
    private final DigitalInput mExitLineBreaker = new DigitalInput(EndEffectorConstants.kExitBreakerPort);
    private final TalonFX mEffectorMotor = new TalonFX(EndEffectorConstants.kMotorPort, "rio");

    private boolean mCoralInPosition = false;
    private boolean mHasCoral = false;
    private boolean mFirstTime = true;
    private double mEjectSpeed = EndEffectorConstants.kDefaultEjectSpeed;

    public EndEffector() {
        mEffectorMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Entrance", entranceDetected());
        SmartDashboard.putBoolean("Exit", exitDetected());
    }

    public Command centerCoral() {
        return runOnce(() -> {
            if (!entranceDetected() && !exitDetected()) {
                mEffectorMotor.set(EndEffectorConstants.kIdleSpeed);
                SmartDashboard.putString("End Effector Branch", "No Coral");
                mCoralInPosition = false;
                mHasCoral = false;
                mFirstTime = true;
            } else if (entranceDetected() && !exitDetected()) {
                mEffectorMotor.set(EndEffectorConstants.kIntakeSpeed);
                mCoralInPosition = false;
                mHasCoral = true;
                SmartDashboard.putString("End Effector Branch", "suck && !vomit");
            } else if (entranceDetected() && exitDetected()) {
                if (mFirstTime) {
                    mEffectorMotor.set(0.05);
                    mCoralInPosition = false;
                    mHasCoral = true;
                    SmartDashboard.putString("End Effector Branch", "firsttime!");

                } else {
                    mEffectorMotor.set(0);
                    mCoralInPosition = true;
                    mHasCoral = true;
                    SmartDashboard.putString("End Effector Branch", "In Position");

                }
            } else if (!entranceDetected() && exitDetected()) {
                mEffectorMotor.set(-EndEffectorConstants.kIntakeSpeed);
                mCoralInPosition = false;
                mHasCoral = true;
                mFirstTime = false;
                SmartDashboard.putString("End Effector Branch", "!suck && vomit");

            }
        }).andThen(Commands.waitSeconds(0.05));
    }

    // getter for mHasCoral
    public boolean hasCoral() {
        return mHasCoral;
    }

    // getter for mCoralInPosition
    public boolean coralInPosition() {
        return mCoralInPosition;
    }

    public boolean entranceDetected() {
        return !mEntranceLineBreaker.get();
    }

    public boolean exitDetected() {
        return !mExitLineBreaker.get();
    }

    // setter for mEjectSpeed
    public void setEjectSpeed(double speed) {
        mEjectSpeed = speed;
    }

    public Command ejectCoral() {
        return runOnce(() -> {
            mHasCoral = false;
            mCoralInPosition = false;
            mFirstTime = true;
            mEffectorMotor.set(mEjectSpeed);
        });
    }

    public Command scoreCoral() {
        return run(() -> {
            mHasCoral = false;
            mCoralInPosition = false;
            mFirstTime = true;
            mEffectorMotor.set(mEjectSpeed);
        });
    }

    /**
     * Waits for the elevator to reach its target position, then ejects the coral
     * and turns off motors
     * 
     * @param elevatorAtHeight lambda that informs the command when the elevator is
     *                         at the target height
     * @return the command sequence to run
     */
    public Command scoreSafe(BooleanSupplier elevatorAtHeight) {
        return Commands.sequence(
                Commands.none().until(elevatorAtHeight),
                ejectCoral(),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> mEffectorMotor.stopMotor(), this));
    }

    public Command scoreL1() {
        return run(() -> {
            mHasCoral = false;
            mCoralInPosition = false;
            mFirstTime = true;
            mEffectorMotor.set(-EndEffectorConstants.kL1EjectSpeed);
        });
    }
}
