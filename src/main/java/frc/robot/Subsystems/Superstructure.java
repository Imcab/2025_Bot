package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Components.AlgaeWrist;
import frc.robot.Subsystems.Components.CoralWrist;
import frc.robot.Subsystems.Components.Elevator;

public class Superstructure extends SubsystemBase{

    public enum SuperstructureGoal{
        IDLE,FEED, RUNL2, RUNL3, RUNL4, 
    }
    
    public Leds leds;
    public CoralWrist wrist;
    public AlgaeWrist algae;
    public Elevator elevator;

    public Superstructure(){

        leds = leds.getIntstance();
        wrist = new CoralWrist();
        algae = new AlgaeWrist();
        elevator = new Elevator();

    }

    @Override
    public void periodic(){
        leds.periodic();
    }

    public void StopALL(){
        wrist.stop();
        elevator.stop();
        algae.stop();
    }

    public boolean wristAtSetpoint(){
        return wrist.atGoal();
    }
    public boolean elevatorAtHeight(){
        return elevator.atGoal();
    }

    public void moveAsynchronous(double meters, double coralWrist, double coralSpeed){
        wrist.requestEater(coralSpeed);
        wrist.requestPosition(coralWrist);
        
    }

}
