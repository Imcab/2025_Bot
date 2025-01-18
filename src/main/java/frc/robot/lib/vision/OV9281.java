package frc.robot.lib.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.geometry.Transform3d;

public class OV9281 {
    String kName;
    Transform3d camToRobot;

    PhotonCamera mCamera;

    PhotonPoseEstimator estimator;

    public OV9281(String kName, Transform3d camToRobot){
        this.kName = kName;
        this.camToRobot = camToRobot;

        mCamera = new PhotonCamera(kName);

        estimator = new PhotonPoseEstimator(null, null, camToRobot);

    }

    public boolean isConnected(){
        return mCamera.isConnected();
    }
    public void setDriverMode(boolean x){
        mCamera.setDriverMode(x);
    }
}
