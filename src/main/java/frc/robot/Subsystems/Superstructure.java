package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Components.CoralWrist;
import frc.robot.Subsystems.Components.Elevator;

public class Superstructure extends SubsystemBase{

    public enum SuperstructureGoal{
        IDLE,FEED, RUNL2, RUNL3, RUNL4, 
    }
    
    public Leds leds;
    public CoralWrist wrist;
    public Elevator elevator;

    public Superstructure(){

        leds = new Leds();
        wrist = new CoralWrist();
        elevator = new Elevator();

    }

    public void StopALL(){
        wrist.stop();
    }

    public boolean wristAtSetpoint(){
        return wrist.isAtGOAL();
    }

    public void moveAsynchronous(double meters, double coralWrist, double coralSpeed){
        wrist.requestEater(coralSpeed);
        wrist.requestPosition(coralWrist);
        
    }

}
