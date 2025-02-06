package frc.robot.lib.vision;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotState;
import frc.robot.lib.vision.LimelightHelpers.LimelightResults;
import frc.robot.lib.vision.VisionConfig.limelight;

public class Limelight {

    public static String kName = VisionConfig.limelight.name;
    LimelightHelpers.PoseEstimate mt2;

    public Limelight(){
    }

    public void blink(){
        LimelightHelpers.setLEDMode_ForceBlink(kName);
    }
    public boolean isConnected(){
        double lastUpdate = RobotController.getFPGATime() - LimelightHelpers.getLatency_Pipeline(kName)/1000;
        return lastUpdate < 250;
    }
    public void LedOn(){
        LimelightHelpers.setLEDMode_ForceOn(kName);
    }

    public void LedOff(){
        LimelightHelpers.setLEDMode_ForceOff(kName);
    }

    public double ty(){
        return LimelightHelpers.getTY(kName);
    }

    public double tagPercentage(){
        return LimelightHelpers.getTA(kName);
    }

    public double tx(){
        return LimelightHelpers.getTX(kName);
    }

    public boolean hasTarget(){
        return LimelightHelpers.getTV(kName) && isConnected();
    }

    public int targets(){
        return LimelightHelpers.getTargetCount(kName);
        
    }
 
    public void update(){

        if (RobotState.isRed()) {
            mt2 = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(kName);
        }

        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kName);
    }

    public boolean hasResults(){
        return mt2.tagCount > 0 && isConnected();
    }

    public boolean hasID(int ID){
        
        LimelightResults results = LimelightHelpers.getLatestResults(kName);

        return hasTarget() && results.valid && results.pipelineID == ID;
    }

    public PoseObservation getObservation(){
        return new PoseObservation(mt2.pose, mt2.timestampSeconds, limelight.trust);
    }

    public double rangeForward(){

        // simple proportional ranging control with Limelight's "ty" value
        // this works best if your Limelight's mount height and target mount height are different.
        // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"

        double method = limelight.useTAforRange ? tagPercentage(): ty();

        double targetingForwardSpeed = method * limelight.forwardKp ;
    
        targetingForwardSpeed *= limelight.TrackMaxSpeed;
        targetingForwardSpeed *= limelight.forwardCoefficient;

        return targetingForwardSpeed;
    }
    public double aimAngular(){
        
        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = tx() * limelight.angularKp;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= limelight.TrackMaxAngularSpeed;

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= limelight.aimCoefficient;

        return targetingAngularVelocity;
    }
    
}
