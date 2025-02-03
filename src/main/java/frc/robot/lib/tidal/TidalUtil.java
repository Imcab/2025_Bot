package frc.robot.lib.tidal;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class TidalUtil {
    
    public static Field getField2025(){
        return new Field(Units.inchesToMeters(690.876), Units.inchesToMeters(317));
    }

    public static boolean isBlue() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    }

    public static Coordinate pose2dToCoordinate(Pose2d pose){
        return new Coordinate(pose);
    }

    

}
