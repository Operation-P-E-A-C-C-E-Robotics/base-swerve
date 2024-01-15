package frc.lib.state;

public abstract class StateMachine <T extends Enum<T>> {
    public abstract void update();
    public abstract void requestState(T state);
    public abstract T getState();
    public abstract boolean isDone();

    public final Runnable action(T state){
        return () -> requestState(state);
    }
}
