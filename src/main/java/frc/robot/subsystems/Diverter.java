package frc.robot.subsystems;
public class Diverter {

    private Diverter () {

    }

    public void setDiverterExtension (double position) {

    }

    public void setDiverterRoller (double speed) {

    }

    public double getDiverterExtension () {
        return 0;
    }
    
    public boolean atSetpoint () {
        return false;
    }

    public boolean detectsNote () {
        return false;
    }

    private static final Diverter instance = new Diverter();
    public static Diverter getInstance(){
        return instance;
    }
}
