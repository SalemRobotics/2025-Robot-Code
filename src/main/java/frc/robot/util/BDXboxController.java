package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Arrays;
import java.util.function.IntPredicate;
import java.util.stream.IntStream;

public class BDXboxController extends CommandXboxController {
  /**
   * Constructs an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public BDXboxController(int port) {
    super(port);
  }

  public boolean getRawButton(int button) {
    return getHID().getRawButton(button);
  }

  public Trigger allMeetPredicate(
      final EventLoop loop, final IntPredicate predicate, final Button... buttons) {
    final IntStream stream = Arrays.asList(buttons).stream().mapToInt(b -> b.value);

    return new Trigger(loop, () -> stream.allMatch(predicate));
  }

  public Trigger allMeetPredicate(final IntPredicate predicate, final Button... buttons) {
    final IntStream stream = Arrays.asList(buttons).stream().mapToInt(b -> b.value);

    return new Trigger(() -> stream.allMatch(predicate));
  }

  public Trigger anyMeetPredicate(
      final EventLoop loop, final IntPredicate predicate, final Button... buttons) {
    final IntStream stream = Arrays.asList(buttons).stream().mapToInt(b -> b.value);

    return new Trigger(loop, () -> stream.anyMatch(predicate));
  }

  public Trigger anyMeetPredicate(final IntPredicate predicate, final Button... buttons) {
    final IntStream stream = Arrays.asList(buttons).stream().mapToInt(b -> b.value);

    return new Trigger(() -> stream.anyMatch(predicate));
  }

  public Trigger all(final EventLoop loop, final Button... buttons) {
    return allMeetPredicate(loop, this::getRawButton, buttons);
  }

  public Trigger all(final Button... buttons) {
    return allMeetPredicate(this::getRawButton, buttons);
  }

  public Trigger any(final EventLoop loop, final Button... buttons) {
    return anyMeetPredicate(loop, this::getRawButton, buttons);
  }

  public Trigger any(final Button... buttons) {
    return anyMeetPredicate(this::getRawButton, buttons);
  }
}
