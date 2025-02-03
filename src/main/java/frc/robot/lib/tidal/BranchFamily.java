package frc.robot.lib.tidal;

public class BranchFamily {
    
    private Branch
    leftBranch,
    rightBranch;

    public enum CompletedType{
        kAny, kBoth, kLeft, kRight
    }

    public enum NearestBranch{
        kLeft, kRight
    }


    private CompletedType kType;
    /**
     * Creates a branch Family consisted in 2 Branches, starting for the left
     * @param branch1
     * @param branch2
     */
    public BranchFamily(CompletedType kType,Branch leftBranch, Branch rightBranch){  
        this.kType = kType;   
        this.leftBranch = leftBranch;
        this.rightBranch = rightBranch;
        
    }

    public Branch leftBranch(){
        return leftBranch;
    }
    public Branch rightBranch(){
        return rightBranch;
    }
    
    public boolean isLeftCompleted(){
        return leftBranch.branchCompleted();
    }

    public boolean isRightCompleted(){
        return rightBranch.branchCompleted();
    }

    public NearestBranch findNearestBranch(Coordinate coordinate){

        if (leftBranch.getCoordinate().distanceToCoordinate(coordinate) < rightBranch.getCoordinate().distanceToCoordinate(coordinate)) {
            return NearestBranch.kLeft;
        }

        return NearestBranch.kRight;
    }

    public boolean familyCompleted(){
        boolean completed;
        switch (kType) {
            case kAny:
            completed = isLeftCompleted() || isRightCompleted();    
                break;
            case kBoth:
            completed = isLeftCompleted() && isRightCompleted();
                break;
            case kLeft:
            completed = isLeftCompleted();
                break;
            case kRight:
            completed = isRightCompleted();
                break;
            default:
            completed = false;
                break;
        }

        return completed;
    }


}
