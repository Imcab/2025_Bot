package frc.robot.lib.tidal;



import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Subsystems.Drive.swerve;
import frc.robot.lib.tidal.TidalConfig.feeders;

public class FeederTracker extends TidalFinder{

    public FeederTracker(swerve drive){
        linkVehicle(drive);
        setNavigationInaccuracy(15);
        setConstraints(TidalConfig.pathGains.mFast);

    }

    public ConditionalCommand left(){
        return navigateToCoordinate(feeders.leftFeederAlign);
    }

    public ConditionalCommand right(){
        return navigateToCoordinate(feeders.rightFeederAlign);
    }
  
    public ConditionalCommand nearest(){
        return new ConditionalCommand(left(), right(), ()-> atLeftFeeder());
    }

    public boolean atLeftFeeder(){
        return isBlue().getAsBoolean() ? TidalConfig.feeders.leftFeederDomainBlue.inRange(lastCoordinate().inY()) : TidalConfig.feeders.leftFeederDomainRed.inRange(lastCoordinate().inY());
    }

    public void stop(){
        stopVehicle();
    }

    @Override
    public void periodic(){

    }

}
