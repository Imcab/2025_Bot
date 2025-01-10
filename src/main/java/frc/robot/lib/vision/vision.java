package frc.robot.lib.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.lib.util.HPPMathLib;

public class vision {

    public class innerFieldSystem{

        public AprilTagFieldLayout mLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        public Transform3d transform = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

        public boolean isRed(){
            return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red);
        }

        public enum FocusMode{
            NONE,RIGHT_CORALSTATION, LEFT_CORALSTATION, REEFZONE, BLUE_BARGEZONE, RED_BARGEZONE, PROCESSOR
        }

        public class AprilTags{ 

            public class rawApriltags {
            
                public static int[] getRawBlueIDS(){
                    int[] IDS = {13,12,18,19,17,20,21,22,16,15,14};
                    return IDS;
                }
                public static int[] getRawRedIDS(){
                    int[] IDS = {2,3,4,5,9,10,11,8,7,6,1};;
                    return IDS;
                }
            }

            public class filteredApriltags {

                public class blueAlliance {
                    
                    public static int[] getRIGHTCORAL(){
                        int[] IDS = {12,16,17,22};
                        return IDS;
                    }
                    public static int[] getLEFTCORAL(){
                        int[] IDS = {13,19,18,20};
                        return IDS;
                    }
        
                    public static int[] getREEFZONE(){
                        int[] IDS = {17,18,19,20,21,22};
                        return IDS;
                    }
                    
                    public static int[] getBARGERZONEBLUE(){
                        int[] IDS = {14,20,21};
                        return IDS;
                    }

                    public static int[] getBARGERZONERED(){
                        int[] IDS = {15,16,22, 21};
                        return IDS;
                    }

                    public static int[] getPROCESSOR(){
                        int[] IDS = {15,16,22};
                        return IDS;
                    }                    
                }
                public class redAlliance {

                    public static int[] getRIGHTCORAL(){
                        int[] IDS = {1,6,7,11};
                        return IDS;
                    }
                    public static int[] getLEFTCORAL(){
                        int[] IDS = {2,9,8,7};
                        return IDS;
                    }
        
                    public static int[] getREEFZONE(){
                        int[] IDS = {7,8,9,10,11,6};
                        return IDS;
                    }
                    
                    public static int[] getBARGERZONEBLUE(){
                        int[] IDS = {3,4,3,9};
                        return IDS;
                    }

                    public static int[] getBARGERZONERED(){
                        int[] IDS = {5,10,11,6};
                        return IDS;
                    }

                    public static int[] getPROCESSOR(){
                        int[] IDS = {3,4,9};
                        return IDS;
                    }
                    
                }
                      
            }
            
        }
            
    } 
  
    public class OV9281 extends innerFieldSystem{
        
        public class OV9281Configuration{
        
            private Boolean drivermode;
            private String name;
            private PoseStrategy bestPoseStrategy, singleTagStrategy;
            private Transform3d robotToCam;
            private Matrix<N3, N1> single;
            private Matrix<N3, N1> multi;

            public OV9281Configuration(String name, PoseStrategy bestPoseStrategy, PoseStrategy singleTagStarStrategy, Transform3d robotToCam, Boolean drivermode, Matrix<N3, N1> multi, Matrix<N3, N1> single){
                this.name = name;
                this.bestPoseStrategy = bestPoseStrategy;
                this.singleTagStrategy = singleTagStarStrategy;
                this.robotToCam = robotToCam;
                this.drivermode = drivermode;
                this.multi = multi; //desv stand
                this.single = single;
            }
            public String getName(){
                return name;
            }
            public Transform3d getrobotToCam(){
                return robotToCam;
            }
            public PoseStrategy getSingleTag(){
                return singleTagStrategy;
            }
            public PoseStrategy getBest(){
                return bestPoseStrategy;
            }
            public Boolean getDriverMode(){
                return drivermode;
            }
            public Matrix<N3, N1> getMulti(){
                multi = VecBuilder.fill(4, 4, 8); // change
                return multi;
            }
            public Matrix<N3, N1> getSingle(){
                single = VecBuilder.fill(0.5, 0.5, 1); // change
                return single;
            }
            public void setMulti(Matrix<N3, N1> stv){
                this.multi = stv;
            }
            public void setSingle(Matrix<N3, N1> stv){
                this.single = stv;
            }      
        }
        
        public PhotonCamera camera;
        public PhotonPoseEstimator estimator;
        public Matrix<N3, N1> curStdDevs; //current
        public OV9281Configuration config;
        public boolean reject;

            
        public OV9281(OV9281Configuration config){
            this.reject = false;
            this.config = config;
            this.camera = new PhotonCamera(config.getName());
            this.estimator = new PhotonPoseEstimator(mLayout, config.getBest(), config.getrobotToCam());

            camera.setDriverMode(config.getDriverMode());

            estimator.setMultiTagFallbackStrategy(config.getSingleTag());
        }
    
        private Optional<EstimatedRobotPose> GlobalPose(){
            Optional<EstimatedRobotPose> estimatedPose = Optional.empty();
            for (var change : camera.getAllUnreadResults()) {
                estimatedPose = estimator.update(change);  
                updateEstimation(estimatedPose, change.getTargets());
            }

           return estimatedPose;
        }

        private void updateEstimation(Optional<EstimatedRobotPose> Pose, List<PhotonTrackedTarget> targets){
            if (Pose.isEmpty()) {
                curStdDevs = config.getSingle(); 

            } else {
            var estStdDevs = config.getSingle(); // desv est estimada
            int aprilCount = 0;
            double Distance = 0; // avg distance

            for (var tgt : targets) {
                var poseTag = estimator.getFieldTags().getTagPose(tgt.getFiducialId()); // pos y or
                if (poseTag.isEmpty()) continue;
                aprilCount++;
                Distance += poseTag.get().toPose2d().getTranslation().getDistance(Pose.get().estimatedPose.toPose2d().getTranslation()); // !

            }

            if (aprilCount == 0){
                curStdDevs = config.getSingle();
            } else {
                Distance /= aprilCount;

                if (aprilCount > 1) estStdDevs = config.getMulti(); // disminuye desv est 

                if (aprilCount == 1 && Distance > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double. MAX_VALUE, Double.MAX_VALUE);
                
                else estStdDevs = estStdDevs.times(1 + (HPPMathLib.Square(Distance)) / 30); // !
                
                curStdDevs = estStdDevs;

                }

            }
            
        }

        public Matrix<N3, N1> getEstimationStdDevs(){
            return curStdDevs;
        }

        public void addVisionMeasurement(SwerveDrivePoseEstimator odometer, double angularVel){
            
            if(Math.abs(angularVel) > 720) // si la velocidad angular es muy alta, ignora las actualizaciones de la odometría
            {
                reject = true;
            }
            if(!reject)
            {
            odometer.addVisionMeasurement(GlobalPose().get().estimatedPose.toPose2d(), GlobalPose().get().timestampSeconds);
            }

        }
        public void rejectPoseUpdate(){
            this.reject = true;
        }

        public boolean isUpdateRejected(){
            return reject;
        }
        public void enable(){
            this.reject = false;
        }

    }

    public class photonSystem extends innerFieldSystem{

        public OV9281[]cams;
        public SwerveDrivePoseEstimator odometer;
        public double gyroRate;

        photonSystem(OV9281[]cams, SwerveDrivePoseEstimator odometer, double gyroRate){
            this.cams = cams;
            this.odometer = odometer;
            this.gyroRate = gyroRate;
        }

        public void updateAll(){
            for(var singleCam : cams){
                singleCam.addVisionMeasurement(odometer, gyroRate);
            }
        }
        public void stop(){
            for(var singleCam : cams){
                singleCam.rejectPoseUpdate();
            }
        }
        public void enable(){
            for(var singleCam : cams){
                singleCam.enable();
            }
        }
    }
    

    public class limelight extends innerFieldSystem{

        public String mName;
        public boolean doRejectUpdate;
        public boolean mDiscard;
        public FocusMode mMode;
        public Boolean enableFocus;

        public limelight(String mName){
            this.mName = mName;
            doRejectUpdate = true;
            enableFocus = false;
            mMode = FocusMode.NONE;
            mDiscard = false;
        }

        public void discardEnemyAllianceTags(boolean discard){
            this.mDiscard = discard;
        }

        public void enableFocus(boolean mode){
            this.enableFocus = mode;
        }
        public void setFocus(FocusMode mFocus){
            if (!enableFocus) {
                mMode = FocusMode.NONE;
            }
            this.mMode = mFocus;   
        }
        public FocusMode getFocus(){
            return mMode;
        }
        public void startFocus(){
            
            Boolean Redcolor = isRed();
            switch (mMode) {
                case NONE:
                    if (mDiscard && Redcolor) {
                        int[] validIDs = AprilTags.rawApriltags.getRawRedIDS();
                        LimelightHelpers.SetFiducialIDFiltersOverride(mName, validIDs);
                    }
                    if (mDiscard && !Redcolor) {
                        int[] validIDs = AprilTags.rawApriltags.getRawBlueIDS();
                        LimelightHelpers.SetFiducialIDFiltersOverride(mName, validIDs);
                    }   
                    break;
                case RIGHT_CORALSTATION:
                    if (mDiscard && Redcolor) {
                        int[] validIDs = AprilTags.filteredApriltags.redAlliance.getRIGHTCORAL();
                        LimelightHelpers.SetFiducialIDFiltersOverride(mName, validIDs);
                    }
                    if (mDiscard && !Redcolor) {
                        int[] validIDs = AprilTags.filteredApriltags.blueAlliance.getRIGHTCORAL();
                        LimelightHelpers.SetFiducialIDFiltersOverride(mName, validIDs);
                    } 
                    break;
                case LEFT_CORALSTATION:
                    if (mDiscard && Redcolor) {
                        int[] validIDs = AprilTags.filteredApriltags.redAlliance.getLEFTCORAL();
                        LimelightHelpers.SetFiducialIDFiltersOverride(mName, validIDs);
                    }
                    if (mDiscard && !Redcolor) {
                        int[] validIDs = AprilTags.filteredApriltags.blueAlliance.getLEFTCORAL();
                        LimelightHelpers.SetFiducialIDFiltersOverride(mName, validIDs);
                    } 
                    break;
                case REEFZONE:
                    if (mDiscard && Redcolor) {
                        int[] validIDs = AprilTags.filteredApriltags.redAlliance.getREEFZONE();
                        LimelightHelpers.SetFiducialIDFiltersOverride(mName, validIDs);
                    }
                    if (mDiscard && !Redcolor) {
                        int[] validIDs = AprilTags.filteredApriltags.blueAlliance.getREEFZONE();
                        LimelightHelpers.SetFiducialIDFiltersOverride(mName, validIDs);
                    } 
                    break;
                case PROCESSOR:
                    if (mDiscard && Redcolor) {
                        int[] validIDs = AprilTags.filteredApriltags.redAlliance.getPROCESSOR();
                        LimelightHelpers.SetFiducialIDFiltersOverride(mName, validIDs);
                    }
                    if (mDiscard && !Redcolor) {
                        int[] validIDs = AprilTags.filteredApriltags.blueAlliance.getPROCESSOR();
                        LimelightHelpers.SetFiducialIDFiltersOverride(mName, validIDs);
                    } 
                    break;
                case BLUE_BARGEZONE:
                    if (mDiscard && Redcolor) {
                        int[] validIDs = AprilTags.filteredApriltags.redAlliance.getBARGERZONEBLUE();
                        LimelightHelpers.SetFiducialIDFiltersOverride(mName, validIDs);
                    }
                    if (mDiscard && !Redcolor) {
                        int[] validIDs = AprilTags.filteredApriltags.blueAlliance.getBARGERZONEBLUE();
                        LimelightHelpers.SetFiducialIDFiltersOverride(mName, validIDs);
                    } 
                    break;
                case RED_BARGEZONE:
                    if (mDiscard && Redcolor) {
                        int[] validIDs = AprilTags.filteredApriltags.redAlliance.getBARGERZONERED();
                        LimelightHelpers.SetFiducialIDFiltersOverride(mName, validIDs);
                    }
                    if (mDiscard && !Redcolor) {
                        int[] validIDs = AprilTags.filteredApriltags.blueAlliance.getBARGERZONERED();
                        LimelightHelpers.SetFiducialIDFiltersOverride(mName, validIDs);
                    } 
                    break;
            
                default:
                    break;
            }
        }
        public double getTX(){
            return LimelightHelpers.getTX(mName);
        }
        public double getTY(){
            return LimelightHelpers.getTY(mName);
        }
        public double getTargetArea(){
            return LimelightHelpers.getTA(mName);
        }
        public boolean hasTarget(){
            return LimelightHelpers.getTV(mName);            
        }
        public void ledBlink(){
            LimelightHelpers.setLEDMode_ForceBlink(mName);
        }
        public void ledOn(){
            LimelightHelpers.setLEDMode_ForceOn(mName);
        }
        public void ledOff(){
            LimelightHelpers.setLEDMode_ForceOff(mName);
        }
        public void changePipeLine(int pipeline){
            LimelightHelpers.setPipelineIndex(mName, pipeline);
        }
        public void getTargetCount(){
            LimelightHelpers.getTargetCount(mName);
        }
        public Pose2d getPose2d(){

            boolean Redcolor  = isRed();

            return Redcolor ? LimelightHelpers.getBotPose2d_wpiRed(mName): LimelightHelpers.getBotPose2d_wpiBlue(mName);
        }
        public void addVisionMeasurement(SwerveDrivePoseEstimator m_poseEstimator, double angularVelocity){
            
            LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            if(Math.abs(angularVelocity) > 720) // si la velocidad angular es muy alta, ignora las actualizaciones de la odometría
            {
                doRejectUpdate = true;
            }
            if(mt2.tagCount == 0)
            {
            doRejectUpdate = true;
            }
            if(!doRejectUpdate)
            {
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            m_poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
            }
        }

        public void rejectPoseUpdate(){
            this.doRejectUpdate = true;
        }

        public boolean isUpdateRejected(){
            return doRejectUpdate;
        }
    }
}

