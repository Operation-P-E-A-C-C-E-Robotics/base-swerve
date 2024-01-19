package frc.lib.state;

public abstract class StateMachine <T extends Enum<T>> {
    /**
     * This is the method that is called every loop to update the state machine.
     * The state machine will handle any automatic state transitions and
     * set it's subsystem output to reflect the current state.
     */
    public abstract void update();

    /**
     * This method is called to request a state change.
     * The change is not guranteed to happen, it can be blocked
     * by the state machine if it cannot safely transition to the new state.
     */
    public abstract void requestState(T state);

    /**
     * get the current state of the state machine
     */
    public abstract T getState();

    /**
     * check if the state machine is in a stable state
     * @return true if the subsystem is in the desired state
     */
    public boolean isDone() {
        return false;
    }

    /**
     * check if the state machine is in a dynamic state
     * dynamic states are states that are not stable and
     * need to be updated every loop
     * @return true if the state machine is in a dynamic state
     */
    public boolean isDynamic() {
        return true;
    }

    /**
     * Get a callback that will request a state change
     * @param state the state to request
     * @return a callback that will request the state change
     */
    public final Runnable action(T state){
        return () -> requestState(state);
    }

    /**
     * Update the state machine with a desired state.
     * Will skip the state transition if the state is already the desired state.
     * Won't update the state machine if it is stable in the requested state.
     * @param state the current state
     */
    public final void updateWithState(T state){
        if(getState() != state){
            requestState(state);
            update();
            return;
        }
        if(isDynamic() || !isDone()){
            update();
        }
    }
}
