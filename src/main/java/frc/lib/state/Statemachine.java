package frc.lib.state;

public interface Statemachine <T extends Enum<T>> {
    public void update();
    public void requestState(T state);
    public T getState();
    public boolean isDone();
}
