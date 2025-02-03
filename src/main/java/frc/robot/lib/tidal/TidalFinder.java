package frc.robot.lib.tidal;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.swerve;
import frc.robot.lib.tidal.TidalConfig.feeders;
import frc.robot.lib.tidal.TidalConfig.pathGains;

//Pathfinder generator
public class TidalFinder{
    
    private Coordinate position;
    private swerve drive;
  
    public TidalFinder(){
        position = new Coordinate(0, 0);
    }

    public void linkRequirements(swerve drive){
        this.drive = drive;
    }

    public BooleanSupplier isBlue(){
        return ()-> TidalUtil.isBlue();
    }
    
    public Command toCoordinate(Coordinate goal, PathConstraints gains){
        Command finder = AutoBuilder.pathfindToPose(goal.toPose2d(), gains);
        finder.addRequirements(drive);
        return finder;
    }

    public Command toRightFeederBlue(){
        return toCoordinate(feeders.rightFeederAlign, pathGains.mdefault);
    }
    public Command toLeftFeederBlue(){
        return toCoordinate(feeders.leftFeederAlign, pathGains.mdefault);
    }
    public Command toRightFeederRed(){
        return toCoordinate(feeders.rightFeederAlignRED, pathGains.mdefault);
    }
    public Command toLeftFeederRed(){
        return toCoordinate(feeders.leftFeederAlignRED, pathGains.mdefault);
    }
 
    public boolean atLeftFeeder(){
        return !TidalUtil.isBlue() ? feeders.leftFeederDomainRed.inRange(position.inY()) : feeders.leftFeederDomainBlue.inRange(position.inY());
    }

    public void uploadReference(Coordinate reference){
        this.position = reference;
    }

    public Coordinate lastCoordinate(){
        return position;
    }


}
