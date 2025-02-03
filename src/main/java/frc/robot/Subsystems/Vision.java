package frc.robot.Subsystems;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.robot.lib.vision.AprilCam;
import frc.robot.lib.vision.Limelight;
import frc.robot.lib.vision.PoseObservation;
import frc.robot.lib.vision.VisionConfig.photonvision;

public class Vision {
    public Limelight limelight;
    public boolean Limerequest = false;
    public AprilCam BL_cam, BR_cam;
    private boolean loop_ONCE = false;

    public Vision(){

        limelight = new Limelight();

        BL_cam = new AprilCam(
        photonvision.backLeft,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        photonvision.robotToBackLeft,
        photonvision.trustBL);

        BR_cam = new AprilCam(
        photonvision.backRight,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        photonvision.robotToBackRight,
        photonvision.trustBR);

        BL_cam.unfilterForDriver(photonvision.driveModeBL);
        BR_cam.unfilterForDriver(photonvision.driveModeBR);

    }

    //Periodic method to call 
    public void periodic(){

        //Loop-Once per loop
        loop_ONCE = false;

        if (!loop_ONCE) {
            
            limelight.update();
            BL_cam.update();
            BR_cam.update();

            loop_ONCE = true;

        }

        if (limelight.hasTarget() && !limeIsRequested()) {
            limelight.blink();
        }else if (limelight.hasTarget() && limeIsRequested()) {
            limelight.LedOn();
        }else{
            limelight.LedOff();
        }


    }
    public boolean BL_hasResults(){
        return BL_cam.hasResults();
    }
    public boolean BR_hasResults(){
        return BR_cam.hasResults();
    }
    public PoseObservation observationBL(){
        return BL_cam.observation();
    }
    public PoseObservation observationBR(){
        return BR_cam.observation();
    }
    public PoseObservation observationLime(){
        return limelight.getObservation();
    }
    public void limeRequest(boolean toggle){
        Limerequest = toggle;
    }
    public boolean limeIsRequested(){
        return Limerequest;
    }
    public boolean lime_hasResults(){
        return limelight.hasResults();
    }
    public double aim(){
        return limelight.aimAngular();
    }
    public double range(){
        return limelight.rangeForward();
    }

}
