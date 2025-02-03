package frc.robot.lib.tidal;

import frc.robot.Constants.ElevatorConstants;

public class Node {

    public enum ReefLevel{
        L1,L2,L3,L4
    }
    
    private ReefLevel level;
    private boolean scored;
    private boolean algaeIncrusted;

    public Node(ReefLevel level, boolean algaeIncrusted){
        this.level = level;
        this.scored = false;
        this.algaeIncrusted = algaeIncrusted;
    }

    public ReefLevel getLevel(){
        return level;
    }

    public void setScored(boolean scored){
        this.scored = scored;
    }

    public boolean isScored(){
        return scored;
    }

    public boolean hasAlgae(){
        return algaeIncrusted;
    }

    public double getHeight(){
        double height;

        switch (level) {
            case L1:
            height = ElevatorConstants.SETPOINT_RETRACT;    
                break;
            case L2:
            height = ElevatorConstants.SETPOINT_L2;
                break;
            case L3:
            height = ElevatorConstants.SETPOINT_L3;
                break;
            case L4:
            height = ElevatorConstants.SETPOINT_L4;
            default:
            height = 0;
                break;
        }

        return height;
    }

}
