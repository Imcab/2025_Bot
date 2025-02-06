package frc.robot.lib.tidal;

import frc.robot.lib.tidal.TidalConfig.reef;

public class ReefTracker extends TidalFinder{

    public ReefTracker(){
        
    }

    public boolean atReef(){

        if (!TidalUtil.isBlue()) {
            return reef.RedReef.atGrid(TidalUtil.pose2dToCoordinate(lastCoordinate().toPose2d()));
        }
        return reef.BlueReef.atGrid(TidalUtil.pose2dToCoordinate(lastCoordinate().toPose2d()));
    }
}
