package frc.robot.util;

import java.util.function.Supplier;
import lombok.AllArgsConstructor;

@AllArgsConstructor
public class Allocated<T> implements Supplier<T> {
  private T value;

  @Override
  public T get() {
    return value;
  }

  public T set(T newValue) {
    final T oldValue = this.value;
    this.value = newValue;

    return oldValue;
  }
}
