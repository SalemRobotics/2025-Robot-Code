package frc.robot.util;

public class Allocated<T> {
    private T value;

    public Allocated(T value) {
        this.value = value;
    }

    public T get() {
        return value;
    }
    public T set(T newValue) {
        T inter = value;
        value = newValue;
        return inter;
    }
}
