package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EndEffector extends SubsystemBase {
    private final DigitalInput mEntranceLineBreaker = new DigitalInput(EndEffectorConstants.kEntranceBreakerPort);
    private final DigitalInput mExitLineBreaker = new DigitalInput(EndEffectorConstants.kExitBreakerPort);

    private boolean mCoralInPosition = false;
    private boolean mAlreadyStopped = false;
    private boolean mHasCoral = false;
    private boolean firsttime = true;
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
        return Commands.runOnce(() -> {

            // if (mEntranceLineBreaker.get() && mExitLineBreaker.get()) {
            //     SmartDashboard.putString("End Effector 1st Branch", "No coral");
            //     // make sure flags are unset
            //     mEffectorMotor.set(EndEffectorConstants.kIdleSpeed);
            //     mCoralInPosition = false;
            //     mHasCoral = false;
            // } else if (mCoralInPosition) {
            //     // don't do anything if there isn't a coral or the coral is in position
            //     SmartDashboard.putString("End Effector 1st Branch", "Skipping");
            //     return;
            // } else if (!mEntranceLineBreaker.get() && mExitLineBreaker.get()) {
            //     if (mAlreadyStopped) {
            //     // start slowing the motor
            //         SmartDashboard.putString("End Effector 1st Branch", "Slowing down");
            //         mEffectorMotor.set(EndEffectorConstants.kIntakeSpeed);
            //         mHasCoral = true;
            //     } else {
            //         SmartDashboard.putString("End Effector 1st Branch", "First brake");
            //         mEffectorMotor.stopMotor();
            //         mAlreadyStopped = true;
            //     }
            // } else if (mEntranceLineBreaker.get() && !mExitLineBreaker.get()) {
            //     // stop the motor temporarily to remove coral momentum
            //     SmartDashboard.putString("End Effector 1st Branch", "Temporary stop");
            //     mEffectorMotor.stopMotor();
            //     mCoralInPosition = true;
            //     mAlreadyStopped = false;
            // }
            if(!suck()&&!vomit()){
                mEffectorMotor.set(EndEffectorConstants.kIdleSpeed);
                SmartDashboard.putString("End Effector Branch", "!suck && !vomit");
                mCoralInPosition = false;
                mHasCoral = false;
                firsttime = true;
            }
            else if(suck()&&!vomit()){
                mEffectorMotor.set(EndEffectorConstants.kIntakeSpeed);
                mCoralInPosition = false;
                mHasCoral = true;
                SmartDashboard.putString("End Effector Branch", "suck && !vomit");

            }
            else if(suck()&&vomit()){
                if(firsttime){
                    mEffectorMotor.set(0.05);
                    mCoralInPosition = false;
                    mHasCoral = true;
                    SmartDashboard.putString("End Effector Branch", "firsttime!");

                }
                else{
                    mEffectorMotor.set(0);
                    mCoralInPosition = true;
                    mHasCoral = true;
                    SmartDashboard.putString("End Effector Branch", "suck && vomit, not first");

                }
            }
            else if(!suck()&&vomit()){
                mEffectorMotor.set(-EndEffectorConstants.kIntakeSpeed);
                mCoralInPosition = false;
                mHasCoral = true;
                firsttime = false;
                SmartDashboard.putString("End Effector Branch", "!suck && vomit");

            }
        }, this).andThen(Commands.waitSeconds(0.1));
                
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
    }

    // getter for mHasCoral
    public boolean hasCoral() {
        return mHasCoral;
    }

    // getter for mCoralInPosition
    public boolean coralInPosition() {
        return mCoralInPosition;
    }

    public boolean suck(){
        return !mEntranceLineBreaker.get();
    }

    public boolean vomit(){
        return !mExitLineBreaker.get();
    }

    // setter for mEjectSpeed
    public void setEjectSpeed(double speed) {
        mEjectSpeed = speed;
    }

    public Command ejectCoral() {
        return Commands.run(() -> {
            mHasCoral = false;
            mCoralInPosition = false;
            firsttime = true;
            mEffectorMotor.set(mEjectSpeed);
        }, this);
    }
}
