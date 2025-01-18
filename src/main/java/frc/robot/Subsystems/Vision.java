package frc.robot.Subsystems;

import frc.robot.lib.vision.Limelight;
import frc.robot.lib.vision.OV9281;
import frc.robot.lib.vision.PoseObservation;

public class Vision {
    public Limelight limelight;
    //public OV9281 photon1, photon2;
    public boolean request = false;
    public Vision(){
        limelight = new Limelight();
    }

    //Periodic method to call 
    public void periodic(){

        if (limelight.hasTarget() && !limeIsRequested()) {
            limelight.blink();
        }else if (limelight.hasTarget() && limeIsRequested()) {
            limelight.LedOn();
        }else{
            limelight.LedOff();
        }

        //starts the vision observation
        //limelight.update();

    }
    public void limeRequest(boolean toggle){
        request = toggle;
    }
    public boolean limeIsRequested(){
        return request;
    }
    public boolean isLimeEmpty(){
        return limelight.isEmpty();
    }
    public PoseObservation getLimeObservation(){
        return limelight.getObservation();
    }
    public double limeTY(){
        return limelight.ty();
    }
    public double limeTX(){
        return limelight.tx();
    }
    public double limeTA(){
        return limelight.tagPercentage();
    }
    public double limeTargetsCount(){
        return limelight.targets();
    }
    public boolean hasTarget(){
        return limelight.hasTarget();
    }
    public double aim(){
        return limelight.aimAngular();
    }
    public double range(){
        return limelight.rangeForward();
    }

}
