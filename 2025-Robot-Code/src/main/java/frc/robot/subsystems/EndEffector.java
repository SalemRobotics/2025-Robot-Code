package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EndEffector extends SubsystemBase {
    private final DigitalInput mEntranceLineBreaker = new DigitalInput(EndEffectorConstants.kEntranceBreakerPort);
    private final DigitalInput mExitLineBreaker = new DigitalInput(EndEffectorConstants.kExitBreakerPort);

    private boolean mCoralInPosition = false;
    private boolean mRepositioningCoral = false;
    private boolean mHasCoral = false;
    private double mEjectSpeed = EndEffectorConstants.kDefaultEjectSpeed;

    private final TalonFX mEffectorMotor = new TalonFX(EndEffectorConstants.kMotorPort, "rio");

    public EndEffector() {
        mEffectorMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Entrance", mEntranceLineBreaker.get());
        SmartDashboard.putBoolean("Exit", mExitLineBreaker.get());
        SmartDashboard.putBoolean("Has Coral", mHasCoral);
        SmartDashboard.putBoolean("Coral In Position", mCoralInPosition);
        SmartDashboard.putNumber("End Effector Speed", mEffectorMotor.get());
    }

    public Command centerCoral() {
        /*
         * Process:
         * 1. Detect a coral at entrance
         * 2. Slow down motors & wait until it is only seen at exit
         * 3. Brake the motors
         * 4. Reverse the motors until the coral is seen at both the exit and the
         * entrance
         * 5. Stop & set the inPosition flag(for AJ controller rumbling when it is safe
         * to raise elevator)
         * 
         * Considerations:
         * - Making this process quicker(adjusting speeds) while not affecting coral's
         * end position
         * - Improving the speed of this command, however it shoudln't be performance
         * intensive already.
         */
        return Commands.run(() -> {
            // false = found coral, true = no coral
            boolean entrance = mEntranceLineBreaker.get();
            boolean exit = mExitLineBreaker.get();

            if (entrance && exit) {
                // if neither breaker sees it we don't have a coral anymore
                SmartDashboard.putString("End Effector Branch", "No coral");
                mCoralInPosition = false;
                mHasCoral = false;
                mEffectorMotor.set(EndEffectorConstants.kIdleSpeed);
            } else if (mCoralInPosition) {
                SmartDashboard.putString("End Effector Branch", "Skip branch");
                // if the coral is in position, we can skip the rest of the loop.
                // TODO: find out if we need to add an extra stopMotor call here
                return;
            } else  if (!entrance && exit) {
                SmartDashboard.putString("End Effector Branch", "Just found");
                // we're just detecting a coral, and slow down the motor
                mHasCoral = true;
                mEffectorMotor.set(EndEffectorConstants.kIntakeSpeed);
            } else if (entrance && !exit) {
                SmartDashboard.putString("End Effector Branch", "Stopping momentum");
                // temporarily stop the motor to invoke braking and get rid of the momentum
                mEffectorMotor.stopMotor();
                mRepositioningCoral = true;
            } if (mRepositioningCoral) {
                if (entrance) {
                    // we continue reversing the coral
                    SmartDashboard.putString("End Effector Branch", "Repositioning");
                    mEffectorMotor.set(-EndEffectorConstants.kAdjustSpeed);
                } else {
                    SmartDashboard.putString("End Effector Branch", "Just in position");
                    // we stop the motor and unset flags
                    mEffectorMotor.stopMotor();
                    mCoralInPosition = true;
                    mRepositioningCoral = false;
                }
            } else {
                SmartDashboard.putString("End Effector Branch", "Else branch");
                mEffectorMotor.set(EndEffectorConstants.kIdleSpeed);
            }

            // OLD IMPLEMENTATION
            // DO NOT DELETE

            // if (!mExitLineBreaker.get() && !mEntranceLineBreaker.get()) {
            // mHasCoral = true;
            // mEffectorMotor.stopMotor();
            // } else if (!mExitLineBreaker.get() && mEntranceLineBreaker.get()) {
            // mEffectorMotor.set(-EndEffectorConstants.kAdjustSpeed);
            // } else if (!mEntranceLineBreaker.get()) {
            // mEffectorMotor.set(EndEffectorConstants.kAdjustSpeed);
            // } else {
            // if (mEntranceLineBreaker.get()) {
            // mHasCoral = false;
            // }
            // mEffectorMotor.set(EndEffectorConstants.kIdleSpeed);
            // // mEffectorMotor.setControl(new VelocityVoltage(20.0));
            // }
        }, this);
    }

    // getter for mHasCoral
    public boolean hasCoral() {
        return mHasCoral;
    }

    // getter for mCoralInPosition
    public boolean coralInPosition() {
        return mCoralInPosition;
    }

    // setter for mEjectSpeed
    public void setEjectSpeed(double speed) {
        mEjectSpeed = speed;
    }

    public Command ejectCoral() {
        return Commands.run(() -> {
            mCoralInPosition = false;
            mHasCoral = false;
            mRepositioningCoral = false;
            mEffectorMotor.set(mEjectSpeed);
        }, this);
    }
}
