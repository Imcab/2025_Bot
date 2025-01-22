package frc.robot.lib.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class AprilCam{
    
    public  PhotonCamera cam;
    public  String kName;
    public Transform3d robotToCam;
    public PhotonPoseEstimator photonPoseEstimator;
    public PoseStrategy strat;
    public Matrix<N3,N1> trust;
     
    public AprilCam(String kName,PoseStrategy strat ,Transform3d robotToCam,Matrix<N3,N1> trust){

        this.kName = kName;
        this.strat = strat;
        this.robotToCam = robotToCam;
        this.trust = trust;
    
        this.cam = new PhotonCamera(kName);

        photonPoseEstimator = new PhotonPoseEstimator(
        VisionConfig.photonvision.layout,
        strat,
        robotToCam);
        
    }

    public void unfilterForDriver(boolean v){
        cam.setDriverMode(v);
    }
    
    public boolean isConnected(){
        return cam.isConnected();
    }

    private Optional<EstimatedRobotPose> globalPose(){
        Optional<EstimatedRobotPose> estVision = Optional.empty();
        for (var change : cam.getAllUnreadResults()) {
            estVision = photonPoseEstimator.update(change);
        }
    
        return estVision;
    }

    public void update(){
        //Only update if its Connected

        if (isConnected()) {
            globalPose();
        }
        //do nothing
    }

    public boolean hasResults(){
        return !globalPose().isEmpty() && isConnected();
    }

    public PoseObservation observation(){
        return new PoseObservation(
        globalPose().get().estimatedPose.toPose2d(),
        globalPose().get().timestampSeconds,
        trust);
    }

}
 