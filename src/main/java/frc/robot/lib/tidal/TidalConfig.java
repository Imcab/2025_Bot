package frc.robot.lib.tidal;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;
import frc.robot.lib.tidal.BranchFamily.CompletedType;
import frc.robot.lib.tidal.TidalReef.ReefPriority;

public class TidalConfig {
    
    public static class pathGains{

        public static PathConstraints mdefault = new PathConstraints(3.0, 3.0,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720));

        public static PathConstraints mSlow = new PathConstraints(2.5, 3.0,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720));

        public static PathConstraints mFast = new PathConstraints(4.5, 4.0,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720));

        public static PathConstraints mCustom(PathConstraints gains){
            return gains;
        }
    }
    public static class feeders {

        public static Domain leftFeederDomainBlue = new Domain(4.3, 8);
        public static Domain leftFeederDomainRed = new Domain(0, 4.2);
        public static Coordinate leftFeederAlign = new Coordinate(1.12, 6.96, 124.88);
        public static Coordinate rightFeederAlign = new Coordinate(1.17, 1.04, -125.75);

        public static Coordinate leftFeederAlignRED = new Coordinate(16.36, 0.93, -55.85);
        public static Coordinate rightFeederAlignRED = new Coordinate(16.40, 6.96, 53.93);
            
    }

    public static class reef {
    
        public static Node lNode = new Node(null, false);
        public static Branch leftBrachBlue = new Branch(null, null, null, null, null, null);
        public static BranchFamily leftFaceBlue = new BranchFamily(CompletedType.kBoth, null, null);
        public static TidalReef tidalBlue = new TidalReef(ReefPriority.LEFT_CENTER_RIGHT, null, null, null);
        //to the center of the reef with our robot at plus-minus 2.20
        public static Grid BlueReef = new Grid(4.5, 4, 220);
        public static Grid RedReef = new Grid(13,4, 220);
        
    }
}
