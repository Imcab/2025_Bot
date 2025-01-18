package frc.robot.lib.vision;

import frc.robot.RobotState;
import frc.robot.lib.vision.LimelightHelpers.LimelightResults;
import frc.robot.lib.vision.LimelightHelpers.PoseEstimate;
import frc.robot.lib.vision.VisionConfig.limelight;

public class Limelight {

    public static String kName = VisionConfig.limelight.name;

    public static PoseEstimate MegaTagEstimate;

    public PoseObservation mObservation;

    public boolean state = false;
    
    public Limelight(){
       
        if (!VisionConfig.limelight.useMegatag2) {
            if (RobotState.isRed()) {
                MegaTagEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed(kName);
            }
            MegaTagEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(kName);
        }

        //asume MegaTag2 is used
        if (RobotState.isRed() && VisionConfig.limelight.useMegatag2) {
            MegaTagEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(kName);
        }
        MegaTagEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kName);
    }

    public void blink(){
        LimelightHelpers.setLEDMode_ForceBlink(kName);
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
        return LimelightHelpers.getTV(kName);
    }
    public int targets(){
        return LimelightHelpers.getTargetCount(kName);
    }
    public LimelightResults results(){
        return LimelightHelpers.getLatestResults(kName);
    }
    
    public void update(){

        boolean doRejectUpdate = false;
        
      if(RobotState.isAngularVelAboveLimits() || MegaTagEstimate.tagCount == 0){
        doRejectUpdate = true;
        setVisionReady(false);
      }
      if(!doRejectUpdate)
      {
        setVisionReady(true);
        sendObservation(new PoseObservation(MegaTagEstimate.pose, MegaTagEstimate.timestampSeconds, limelight.trust));
      }
    }

    private void setVisionReady(boolean v){
        this.state = v;
    }
    public boolean isEmpty(){
        return state;
    }

    private void sendObservation(PoseObservation observation){
        this.mObservation = observation;
    }

    public PoseObservation getObservation(){
        return mObservation;
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
