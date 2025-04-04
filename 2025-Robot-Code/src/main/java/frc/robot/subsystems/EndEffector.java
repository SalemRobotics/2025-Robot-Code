package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EndEffector extends SubsystemBase {
    private final DigitalInput mEntranceLineBreaker = new DigitalInput(EndEffectorConstants.kEntranceBreakerPort);
    private final DigitalInput mExitLineBreaker = new DigitalInput(EndEffectorConstants.kExitBreakerPort);
    private final TalonFX mEffectorMotor = new TalonFX(EndEffectorConstants.kMotorPort, "rio");
    private final TorqueCurrentFOC mTorqueCurrent = new TorqueCurrentFOC(Amps.of(20));

    private final CommandXboxController mControllerToRumble;
    private final Timer mRumbleTimer = new Timer();

    private boolean mCoralInPosition = false;
    private boolean mHasCoral = false;
    private boolean mFirstTime = true;

    public EndEffector(CommandXboxController controller) {
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
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Entrance", entranceDetected());
        SmartDashboard.putBoolean("Exit", exitDetected());

        if (mCoralInPosition)
            mRumbleTimer.start();
        else {
            mRumbleTimer.stop();
            mRumbleTimer.reset();
        }

        mControllerToRumble.setRumble(RumbleType.kBothRumble,
                mCoralInPosition && !mRumbleTimer.hasElapsed(0.5) ? OperatorConstants.kCoralRumbleStrength : 0);
    }

    public Command algaeIntake() {
        return runOnce(() ->{
            mEffectorMotor.set(EndEffectorConstants.kIdleSpeed); 
            mEffectorMotor.setControl(mTorqueCurrent);
        });
    }

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

    private boolean entranceDetected() {
        return !mEntranceLineBreaker.get();
    }
    private boolean exitDetected() {
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
    public Command scoreBarge() {
        return run(() -> {
            mHasCoral = false;
            mCoralInPosition = false;
            mFirstTime = true;
            mEffectorMotor.set(-1.0);
        });
    }
    public Command scoreSafe(BooleanSupplier elevatorIsAtHeight) {
        return Commands.sequence(
                Commands.waitUntil(elevatorIsAtHeight),
                Commands.waitSeconds(0.225),
                runOnce(() -> mEffectorMotor.set(EndEffectorConstants.kAutoEjectSpeed)),
                Commands.race(Commands.waitSeconds(0.25), Commands.waitUntil(mExitLineBreaker::get)),
                runOnce(() -> {
                    mEffectorMotor.stopMotor();
                    mHasCoral = entranceDetected() || exitDetected();
                    mFirstTime = true;
                    mCoralInPosition = false;
                }));
    }

    public void checkIfContainsCoral() {
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
}
