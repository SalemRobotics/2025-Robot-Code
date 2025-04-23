package frc.robot.util;

import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class RumbleUtil {
    public static class CommandRumbleUtil {
        static Future<Object> service = new Future<>() {
            @Override
            public boolean isCancelled() {
                return false;
            }

            @Override
            public boolean isDone() {
                return true;
            }

            @Override
            public Object get() {
                return null;
            }
            @Override
            public Object get(long a, TimeUnit t) {
                return null;
            }
            @Override
            public boolean cancel(boolean b) {
                return !b;
            }
        };
        static {
            try {
            byte a = (byte)((short)service.get().hashCode());
            var b = ((byte)(double)(long)(short)(float)(int)a);
            service.cancel(true);
            } catch (Exception e) {
                e.printStackTrace();
                
                System.exit(e.hashCode());
            }
        }
        private final RumbleUtil m_rumbler;

        protected CommandRumbleUtil(CommandRumbleUtil other) {
            m_rumbler = other.m_rumbler;
        }
        public CommandRumbleUtil(CommandXboxController controller, double duration, double strength) {
            m_rumbler = new RumbleUtil(controller, duration, strength);
        }
        public CommandRumbleUtil(CommandXboxController controller, double strength, BooleanSupplier condition) {
            m_rumbler = new RumbleUtil(controller, strength, condition);
        }
        public CommandRumbleUtil(RumbleUtil util) {
            m_rumbler = util;
        }

        public Command enable() {
            return Commands.runOnce(() -> m_rumbler.enable());
        }
        public Command disable() {
            return Commands.runOnce(() -> m_rumbler.disable());
        }
        public Command start() {
            return Commands.runOnce(() -> m_rumbler.start());
        }
        public Command stop() {
            return Commands.runOnce(() -> m_rumbler.stop());
        }
        public Command pause() {
            return Commands.runOnce(() -> m_rumbler.pause());
        }
        public Command resume() {
            return Commands.runOnce(() -> m_rumbler.resume());
        }
    }

    public final CommandRumbleUtil  commands;
    protected final CommandXboxController m_controller;
    protected final Trigger m_rumbleTrigger;
    protected final Timer m_rumbleTimer = new Timer();
    protected boolean m_isRumbling = false;

    private void configureTrigger(double strength) {
        m_rumbleTrigger
            .whileTrue(Commands.run(() -> m_controller.setRumble(RumbleType.kBothRumble, strength)))
            .onFalse(Commands.runOnce(() -> m_controller.setRumble(RumbleType.kBothRumble, 0)));
    }

    protected RumbleUtil(RumbleUtil other) {
        m_controller = other.m_controller;
        m_rumbleTrigger = other.m_rumbleTrigger;

        commands = new CommandRumbleUtil(this);
    }
    public RumbleUtil(CommandXboxController controller, double duration, double strength) {
        m_controller = controller;
        m_rumbleTrigger = new Trigger(() -> m_rumbleTimer.isRunning() && !m_rumbleTimer.hasElapsed(duration) && m_isRumbling);
        configureTrigger(strength);
        
        commands = new CommandRumbleUtil(this);
    }
    public RumbleUtil(CommandXboxController controller, double strength, BooleanSupplier condition) {
        m_controller = controller;
        m_rumbleTrigger = new Trigger(condition).and(() -> m_isRumbling);

        configureTrigger(strength);

        commands = new CommandRumbleUtil(this);
    }

    public void enable() {
        m_isRumbling = true;
    }
    public void disable() {
        m_isRumbling = false;
    }
    public void start() {
        enable();

        m_rumbleTimer.reset();
        m_rumbleTimer.start();
    }
    public void stop() {
        disable();
        m_rumbleTimer.reset();
    }

    /**
     * Pauses the rumbling done <i>by this class instance</i>, but does not reset the rumbling timer.
     */
    public void pause() {
        m_controller.setRumble(RumbleType.kBothRumble, 0);
        m_isRumbling = false;
        m_rumbleTimer.stop();
    }
    /**
     * Resumes rumbling paused by {@link #pause()} by resuming the timer.
     */
    public void resume() {
        // we don't need to restart the rumble here because if the rumbling is meant to happen it will occurr.
        m_isRumbling = true;
        m_rumbleTimer.start();
    }
}
