package frc.robot.lib.tidal;

public class TidalReef {
    private BranchFamily leftFace;
    private BranchFamily centerFace;
    private BranchFamily rightFace;

    public enum ReefPriority{
        RIGHT_CENTER_LEFT, LEFT_CENTER_RIGHT, CENTER_RIGHT_LEFT, CENTER_LEFT_RIGHT;
    }

    public enum ReefTracking{
        kCenter, kLeft, kRight, kNone;
    }

    private ReefTracking track;
    private ReefPriority priority;

    public TidalReef(ReefPriority priority, BranchFamily leftFace, BranchFamily centerFace, BranchFamily rightFace){
        this.leftFace = leftFace;
        this.centerFace = centerFace;
        this.rightFace = rightFace;
        this.priority = priority;
    }

    public BranchFamily leftFace(){
        return leftFace;
    }

    public BranchFamily centerFace(){
        return centerFace;
    }

    public BranchFamily rightFace(){
        return rightFace;
    }

    public void periodic(){
        switch (priority) {
            case RIGHT_CENTER_LEFT:
            if (!rightFace.familyCompleted() && !centerFace.familyCompleted() && !leftFace.familyCompleted()) {
                track = ReefTracking.kRight;
            }else if (rightFace.familyCompleted() && !centerFace.familyCompleted() && !leftFace.familyCompleted()) {
                track = ReefTracking.kCenter;
            }else if (rightFace.familyCompleted() && centerFace.familyCompleted() && !leftFace.familyCompleted()) {
                track = ReefTracking.kLeft;
            }else{
                track = ReefTracking.kNone;
            }
            break;

            case LEFT_CENTER_RIGHT:
            if (!leftFace.familyCompleted() && !centerFace.familyCompleted() && !rightFace.familyCompleted()) {
                track = ReefTracking.kLeft;
            }else if (!rightFace.familyCompleted() && !centerFace.familyCompleted() && leftFace.familyCompleted()) {
                track = ReefTracking.kCenter;
            }else if (!rightFace.familyCompleted() && centerFace.familyCompleted() && leftFace.familyCompleted()) {
                track = ReefTracking.kRight;
            }else{
                track = ReefTracking.kNone;
            }
            break;

            case CENTER_LEFT_RIGHT:
            if (!leftFace.familyCompleted() && !centerFace.familyCompleted() && !rightFace.familyCompleted()) {
                track = ReefTracking.kCenter;
            }else if (!rightFace.familyCompleted() && centerFace.familyCompleted() && !leftFace.familyCompleted()) {
                track = ReefTracking.kLeft;
            }else if (!rightFace.familyCompleted() && centerFace.familyCompleted() && leftFace.familyCompleted()) {
                track = ReefTracking.kRight;
            }else{
                track = ReefTracking.kNone;
            }
            break;

            case CENTER_RIGHT_LEFT:
            if (!leftFace.familyCompleted() && !centerFace.familyCompleted() && !rightFace.familyCompleted()) {
                track = ReefTracking.kCenter;
            }else if (!rightFace.familyCompleted() && centerFace.familyCompleted() && !leftFace.familyCompleted()) {
                track = ReefTracking.kRight;
            }else if (rightFace.familyCompleted() && centerFace.familyCompleted() && !leftFace.familyCompleted()) {
                track = ReefTracking.kLeft;
            }else{
                track = ReefTracking.kNone;
            }
            break;

            default:
                break;
        }
    }

    public ReefTracking getCurrentTrack(){
        return track;
    }

    public boolean reefCompleted(){
        return rightFace.familyCompleted() && centerFace.familyCompleted() && leftFace.familyCompleted();
    }
    
}
