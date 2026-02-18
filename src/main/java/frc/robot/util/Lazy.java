package frc.robot.util;

import java.util.function.Supplier;

public final class Lazy<T> implements Supplier<T> {
  private T value = null;
  private final Supplier<T> supplier;

  public Lazy(Supplier<T> supplier) {
    this.supplier = supplier;
  }

  @Override
  public T get() {
    if (value == null) {
      value = supplier.get();
    }

    return value;
  }
}
