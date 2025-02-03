package frc.robot.lib.tidal;

public class Branch {

    private Node l1;
    private Node l2;
    private Node l3;
    private Node l4;

    public enum BranchPriority{
        ALL_LEVELS, L3_TO_L4, L2_TO_L4, L4, L3, L2, L1, ANY;
    }


    private BranchPriority mprior;
    private Coordinate position;
 
    public Branch(BranchPriority mprior, Coordinate position, Node l1, Node l2, Node l3, Node l4){
        this.mprior = mprior;
        this.l1 = l1;
        this.l2 = l2;
        this.l3 = l3;
        this.l4  =l4;
    }

    public Coordinate getCoordinate(){
        return position;
    }

    public boolean atBranch(Coordinate coordinate){
        return coordinate.equals(position, 25);
    }

    public Node getL1(){
        return l1;
    }

    public Node getL2(){
        return l2;
    }

    public Node getL3(){
        return l3;
    }

    public Node getL4(){
        return l4;
    }


    public boolean branchCompleted(){
        boolean completed;

        switch (mprior) {
            case L4:
            completed = l4.isScored();    
                break;
            case L3:
            completed = l3.isScored();
                break;
            case L2:
            completed = l2.isScored();
                break;
            case L1:
            completed = l1.isScored();
                break;
            case ALL_LEVELS:
            completed = 
            l1.isScored() &&
            l2.isScored() &&
            l3.isScored() &&
            l4.isScored();
                break;
            case L2_TO_L4:
            completed = 
            l2.isScored() &&
            l3.isScored() &&
            l4.isScored();
                break;
            case L3_TO_L4:
            completed = 
            l3.isScored() &&
            l4.isScored();
                break;
            case ANY:
            completed = 
            l1.isScored() ||
            l2.isScored() ||
            l3.isScored() ||
            l4.isScored();
            default:
            completed = false;
                break;
        }

        return completed;
    }
}
