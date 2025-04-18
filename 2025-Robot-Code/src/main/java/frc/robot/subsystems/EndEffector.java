package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.BooleanSupplier;
import java.lang.Runnable;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EndEffector extends SubsystemBase {
    private final DigitalInput mEntranceLineBreaker = new DigitalInput(EndEffectorConstants.kEntranceBreakerPort);
    private final DigitalInput mExitLineBreaker = new DigitalInput(EndEffectorConstants.kExitBreakerPort);
    private final TalonFX mEffectorMotor = new TalonFX(EndEffectorConstants.kMotorPort, "rio");

    private final TorqueCurrentFOC mAlgaeCurrent = new TorqueCurrentFOC(Amps.of(20));

    private boolean mCoralInPosition = false;
    private boolean mHasCoral = false;
    private boolean mFirstTime = true;

    private final CommandXboxController mControllerToRumble;
    private BooleanSupplier mRumbleEnabled = () -> false;

    private final Timer mInPositionRumbleTimer = new Timer();
    private final Timer mFirstDetectedRumbleTimer = new Timer();
    private final Trigger mInPositionRumbleTrigger = new Trigger(() -> mInPositionRumbleTimer.isRunning()
            && !mInPositionRumbleTimer.hasElapsed(1) && mRumbleEnabled.getAsBoolean());
    private final Trigger mFirstDetectedRumbleTrigger = new Trigger(() -> mFirstDetectedRumbleTimer.isRunning()
            && !mFirstDetectedRumbleTimer.hasElapsed(0.5) && mFirstTime && mRumbleEnabled.getAsBoolean());

    public EndEffector(CommandXboxController controller, BooleanSupplier rumbleEnabled) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.withSupplyCurrentLimit(70);
        config.CurrentLimits.withStatorCurrentLimit(120);

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = mEffectorMotor.getConfigurator().apply(config);
            if (status.isOK())
                break;
        }
        if (!status.isOK())
            System.err.println("Failed to configure algae remover motor: " + status.toString());

        mEffectorMotor.setNeutralMode(NeutralModeValue.Brake);
        mControllerToRumble = controller;
        mRumbleEnabled = rumbleEnabled;

        mInPositionRumbleTrigger
                .whileTrue(Commands.run(() -> mControllerToRumble.setRumble(RumbleType.kBothRumble,
                        OperatorConstants.kCoralRumbleStrength)))
                .onFalse(Commands.runOnce(() -> mControllerToRumble.setRumble(RumbleType.kBothRumble, 0)));
        mFirstDetectedRumbleTrigger
                .whileTrue(Commands.run(() -> mControllerToRumble.setRumble(RumbleType.kBothRumble,
                        OperatorConstants.kCoralRumbleStrength / 2)))
                .onFalse(Commands.runOnce(() -> mControllerToRumble.setRumble(RumbleType.kBothRumble, 0)));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Entrance", entranceDetected());
        SmartDashboard.putBoolean("Exit", exitDetected());

        if (mCoralInPosition)
            mInPositionRumbleTimer.start();
        else {
            mInPositionRumbleTimer.stop();
            mInPositionRumbleTimer.reset();
        }
        if (entranceDetected() && !exitDetected() && mFirstTime)
            mFirstDetectedRumbleTimer.start();
        else {
            mFirstDetectedRumbleTimer.stop();
            mFirstDetectedRumbleTimer.reset();
        }
    }

    public Command algaeIntake() {
        return runOnce(() -> {
            mEffectorMotor.set(EndEffectorConstants.kIdleSpeed);
            mEffectorMotor.setControl(mAlgaeCurrent);
        });
    }

    /**
     * Processes a coral in autonomous or teleop
     * @param inAuto Whether the given command is to run in teleop or autonomous
     * @return The command to run in the provided mode.
     */
    public Command coralIntake(boolean inAuto) {
        final Runnable intake = () -> {
            final boolean entrance = entranceDetected();
            final boolean exit = exitDetected();

            mHasCoral = entrance || exit;
            if (!entrance && !exit) {
                mEffectorMotor.set(EndEffectorConstants.kIdleSpeed);
                mCoralInPosition = false;
                mFirstTime = true;
            } else if (entrance && exit) {
                if (mFirstTime) {
                    mEffectorMotor.set(EndEffectorConstants.kIntakeSpeed / 2);
                    mCoralInPosition = false;
                } else {
                    mEffectorMotor.stopMotor();
                    mCoralInPosition = true;
                }
            } else if (entrance && !exit) {
                mEffectorMotor.set(EndEffectorConstants.kIntakeSpeed);
                mCoralInPosition = false;
            } else if (!entrance && exit) {
                mEffectorMotor.set(-EndEffectorConstants.kDriveBackSpeed);
                mCoralInPosition = false;
                mFirstTime = false;
            } else {
                System.err.println("End effector encountered an invalid state");
            }
        };
        if (inAuto) {
            return run(intake).finallyDo(() -> {
                mEffectorMotor.stopMotor();
                mCoralInPosition = false;
                mHasCoral = false;
                mFirstTime = true;
            });
        } else {
            return runOnce(intake).andThen(Commands.waitSeconds(0.04));
        }
    }

    /**
     * @deprecated This command has been deprecated in favor of {@link #coralIntake(boolean)}
     * @return The command to run or be scheduled
     */
    @Deprecated(forRemoval = true)
    public Command teleIntake() {
        return runOnce(() -> {
            if (!entranceDetected() && !exitDetected()) {
                mEffectorMotor.set(EndEffectorConstants.kIdleSpeed);
                SmartDashboard.putString("End Effector Branch", "No Coral");
                mCoralInPosition = false;
                mHasCoral = false;
                mFirstTime = true;
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
            } else if (entranceDetected() && !exitDetected()) {
                mEffectorMotor.set(EndEffectorConstants.kIntakeSpeed);
                mCoralInPosition = false;
                mHasCoral = true;
                SmartDashboard.putString("End Effector Branch", "Only At Entrance");
            } else if (!entranceDetected() && exitDetected()) {
                mEffectorMotor.set(-EndEffectorConstants.kDriveBackSpeed);
                mCoralInPosition = false;
                mHasCoral = true;
                mFirstTime = false;
                SmartDashboard.putString("End Effector Branch", "Only At Exit");
            } else {
                SmartDashboard.putString("End Effector Branch", "Invalid State");
            }
        }).andThen(Commands.waitSeconds(0.05));
    }

    /**
     * @deprecated This command has been deprecated in favor of {@link #coralIntake(boolean)}
     * @return The command to run or be scheduled
     */
    @Deprecated(forRemoval = true)
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
            mEffectorMotor.set(ejectFast.getAsBoolean() ? EndEffectorConstants.kFastEjectSpeed
                    : EndEffectorConstants.kDefaultEjectSpeed);
        });
    }

    public Command scoreBloop() {
        return run(() -> {
            mHasCoral = false;
            mCoralInPosition = false;
            mFirstTime = true;
            mEffectorMotor.set(EndEffectorConstants.kBloopSpeed);
        });
    }

    public Command scoreL1() {
        return run(() -> {
            mHasCoral = false;
            mCoralInPosition = false;
            mFirstTime = true;
            mEffectorMotor.set(EndEffectorConstants.kL1EjectSpeed);
        });
    }

    public Command scoreBarge() {
        return run(() -> {
            mHasCoral = false;
            mCoralInPosition = false;
            mFirstTime = true;
            mEffectorMotor.set(-EndEffectorConstants.kAlgaeBargeSpeed);
        });
    }

    public Command scoreProcessor() {
        return run(() -> {
            mHasCoral = false;
            mCoralInPosition = false;
            mFirstTime = true;
            mEffectorMotor.set(-EndEffectorConstants.kAlgaeProcessorSpeed);
        });
    }

    public Command scoreSafe(BooleanSupplier elevatorIsAtHeight) {
        return Commands.sequence(
                Commands.waitUntil(elevatorIsAtHeight),
                Commands.waitSeconds(0.15),
                runOnce(() -> mEffectorMotor.set(EndEffectorConstants.kAutoEjectSpeed)),
                Commands.race(Commands.waitSeconds(0.2), Commands.waitUntil(mExitLineBreaker::get)),
                runOnce(() -> {
                    mEffectorMotor.stopMotor();
                    mHasCoral = entranceDetected() || exitDetected();
                    mFirstTime = true;
                    mCoralInPosition = false;
                }));
    }

    public void enableInit() {
        if (entranceDetected() && exitDetected()) {
            mFirstTime = false;
            mCoralInPosition = true;
            mHasCoral = true;
        } else {
            mHasCoral = entranceDetected() || exitDetected();
            mFirstTime = true;
            mCoralInPosition = false;
        }
    }

    public Command autoIntakeFast() {
        return Commands.race(Commands.waitSeconds(0.4), Commands.waitUntil(() -> exitDetected()), autoIntake());
    }

    public Command autoPostIntake(Command elevatorl3) {
        return autoIntake().alongWith(Commands.waitUntil(this::exitDetected).andThen(elevatorl3));
    }
}
