package frc.robot.util;

import java.util.function.Supplier;
import lombok.val;

public final class Statics {
  /** An interface that represents a function that <em>may</em> throw any exception. */
  @FunctionalInterface
  public interface ThrowingSupplier<T> {
    public T get() throws Exception;
  }

  public static final <T> T initOrDefault(ThrowingSupplier<T> init, Supplier<T> defaultSupplier) {
    try {
      return init.get();
    } catch (Exception e) {
      val throwable = e.fillInStackTrace();

      System.err.println("=".repeat(50) + "\nError occurred: " + throwable.getMessage());
      throwable.printStackTrace();
      System.err.println("Not a critical error, robot code continues" + "=".repeat(50));
    }

    return defaultSupplier.get();
  }
}
