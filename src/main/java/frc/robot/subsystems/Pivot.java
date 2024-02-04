package frc.robot.subsystems;
public class Pivot {
    private Pivot () {

    }

    public void setPivotPosition (double position) {

    }

    public double getPivotPosition () {
        return 0;
    }

    public boolean atSetpoint () {
        return false;
    }

    private static final Pivot instance = new Pivot();
    public static Pivot getInstance(){
        return instance;
    }
}
