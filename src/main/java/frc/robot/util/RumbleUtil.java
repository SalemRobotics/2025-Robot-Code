package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RumbleUtil {
    private final CommandXboxController mController;
    private final Timer mRumbleTimer = new Timer();
    private final double mRumbleStrength;
    private final double mDuration;
    private final Trigger mRumbleTrigger;
    private boolean mRumbling = false;

    public RumbleUtil(CommandXboxController controller, double time, double duration, double strength) {
        mController = controller;
        mDuration = duration;
        mRumbleStrength = strength;

        mRumbleTrigger = new Trigger(() -> mRumbleTimer.isRunning() && !mRumbleTimer.hasElapsed(mDuration));
        mRumbleTrigger.whileTrue(Commands.run(() -> mController.setRumble(RumbleType.kBothRumble, mRumbleStrength)))
                .onFalse(Commands.runOnce(() -> mController.setRumble(RumbleType.kBothRumble, 0)));
    }
    public RumbleUtil(CommandXboxController controller, double time, double strength, BooleanSupplier condition) {
        mController = controller;
        mDuration = -1;
        mRumbleStrength = strength;
        
        mRumbleTrigger = new Trigger(() -> condition.getAsBoolean() && mRumbling);
        mRumbleTrigger.whileTrue(Commands.run(() -> mController.setRumble(RumbleType.kBothRumble, mRumbleStrength)))
                .onFalse(Commands.runOnce(() -> mController.setRumble(RumbleType.kBothRumble, 0)));
    }

    public void start() {
        mRumbling = true;
        mRumbleTimer.start();
    }

    public void reset() {
        mRumbling = false;
        mRumbleTimer.stop();
        mRumbleTimer.reset();
    }
}