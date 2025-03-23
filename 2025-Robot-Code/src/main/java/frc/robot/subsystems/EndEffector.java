package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EndEffector extends SubsystemBase {
    private final DigitalInput mEntranceLineBreaker = new DigitalInput(EndEffectorConstants.kEntranceBreakerPort);
    private final DigitalInput mExitLineBreaker = new DigitalInput(EndEffectorConstants.kExitBreakerPort);
    private final TalonFX mEffectorMotor = new TalonFX(EndEffectorConstants.kMotorPort, "rio");
    private final CommandXboxController mControllerToRumble;
    private final Timer mRumbleTimer = new Timer();

    private boolean mCoralInPosition = false;
    private boolean mHasCoral = false;
    private boolean mFirstTime = true;

    public EndEffector(CommandXboxController controller) {
        mEffectorMotor.setNeutralMode(NeutralModeValue.Brake);
        mControllerToRumble = controller;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Entrance", entranceDetected());
        SmartDashboard.putBoolean("Exit", exitDetected());

        if (mCoralInPosition)
            mRumbleTimer.start();
        else
            mRumbleTimer.reset();

        mControllerToRumble.setRumble(RumbleType.kBothRumble,
                mCoralInPosition && !mRumbleTimer.hasElapsed(0.5) ? OperatorConstants.kRumbleStrength : 0);
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
                    mEffectorMotor.set(EndEffectorConstants.kIntakeSpeed / 2);
                    mCoralInPosition = false;
                    mHasCoral = true;
                    SmartDashboard.putString("End Effector Branch", "First Time!");
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
    public Command autoIntake() {
        return run(() -> {
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
                    mEffectorMotor.set(EndEffectorConstants.kIntakeSpeed / 2);
                    mCoralInPosition = false;
                    mHasCoral = true;
                    SmartDashboard.putString("End Effector Branch", "First Time!");
                } else {
                    mEffectorMotor.stopMotor();
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
        }).finallyDo(() -> {
            mEffectorMotor.stopMotor();
            mCoralInPosition = false;
            mHasCoral = false;
            mFirstTime = true;
        });
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

    public Command autoScoreCoral() {
        return runOnce(() -> {
            mHasCoral = false;
            mCoralInPosition = false;
            mFirstTime = true;
            mEffectorMotor.set(EndEffectorConstants.kFastEjectSpeed);
        });
    }

    public Command scoreCoral(BooleanSupplier ejectFast) {
        return run(() -> {
            mHasCoral = false;
            mCoralInPosition = false;
            mFirstTime = true;
            mEffectorMotor.set(ejectFast.getAsBoolean() ? EndEffectorConstants.kFastEjectSpeed : EndEffectorConstants.kDefaultEjectSpeed);
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
                autoScoreCoral(),
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
