package frc.robot.lib.tidal;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Drive.swerve;

//Pathfinder generator
public class TidalFinder extends SubsystemBase{
    
    private Coordinate goal;
    private swerve drive;
    private PathConstraints gains;
    private double acc;
  
    public TidalFinder(){
        goal = new Coordinate(0, 0);
        
    }

    public void linkVehicle(swerve drive){
        if(this.drive != null){
            throw new IllegalStateException("Vehicle already linked!");
        }

        this.drive = drive;
    }

    public void setNavigationInaccuracy(double percentage){
        this.acc = percentage;
    }

    public BooleanSupplier isBlue(){
        return ()-> TidalUtil.isBlue();
    }

    public void setConstraints(PathConstraints gains){
        this.gains = gains;
    }
    
    private Command toSingleCoordinate(Coordinate goal, PathConstraints gainsCustom){
        Command finder = AutoBuilder.pathfindToPose(goal.toPose2d(), gainsCustom)
        .beforeStarting(()-> this.goal = goal).onlyWhile(()->!atGoal()).finallyDo(()->stopVehicle());
        finder.addRequirements(drive);
        return finder;
    }

    private Command toSingleCoordinate(Coordinate goal){
        Command finder = AutoBuilder.pathfindToPose(goal.toPose2d(), gains)
        .beforeStarting(()-> this.goal = goal).onlyWhile(()->!atGoal()).finallyDo(()->stopVehicle());
        finder.addRequirements(drive);
        return finder;
    }

    private Command toSingleCoordinate(Coordinate goal, PathConstraints gainsCustom, Runnable init, BooleanSupplier onlyWhile, Runnable atEnd){
        Command finder = AutoBuilder.pathfindToPose(goal.toPose2d(), gainsCustom).beforeStarting(()-> this.goal = goal)
        .beforeStarting(init).onlyWhile(onlyWhile).finallyDo(atEnd);
        finder.addRequirements(drive);
        return finder;
    }

    private Command toSingleCoordinate(Coordinate goal, PathConstraints gainsCustom, Runnable init, BooleanSupplier onlyWhile, Runnable atEnd, Time time){
        Command finder = AutoBuilder.pathfindToPose(goal.toPose2d(), gainsCustom).beforeStarting(()-> this.goal = goal)
        .beforeStarting(init).onlyWhile(onlyWhile).finallyDo(atEnd).withTimeout(time);
        finder.addRequirements(drive);
        return finder;
    }

    private Command toSingleCoordinate(Coordinate goal, PathConstraints gainsCustom, Runnable init, BooleanSupplier onlyWhile, Runnable atEnd, Command other){
        Command finder = AutoBuilder.pathfindToPose(goal.toPose2d(), gainsCustom).beforeStarting(()-> this.goal = goal)
        .beforeStarting(init).onlyWhile(onlyWhile).finallyDo(atEnd).deadlineFor(other);
        finder.addRequirements(drive);
        return finder;
    }

    

    public ConditionalCommand navigateToCoordinate(Coordinate coordinate, PathConstraints gains){

        return new ConditionalCommand(toSingleCoordinate(coordinate, gains), toSingleCoordinate(TidalUtil.coordinateFlip(coordinate), gains), isBlue());

    }

    public ConditionalCommand navigateToCoordinate(Coordinate coordinate){

        return new ConditionalCommand(toSingleCoordinate(coordinate), toSingleCoordinate(TidalUtil.coordinateFlip(coordinate)), isBlue());

    }

    public ConditionalCommand simplePathFindingCommand(Coordinate coordinate, PathConstraints gains, Runnable init, BooleanSupplier onlyWhile, Runnable atEnd){
        return new ConditionalCommand(toSingleCoordinate(coordinate, gains, init, onlyWhile, atEnd),
         toSingleCoordinate(TidalUtil.coordinateFlip(coordinate), gains, init, onlyWhile, atEnd), isBlue());
    }

    public ConditionalCommand timerPathFindingCommand(Coordinate coordinate, PathConstraints gains, Runnable init, BooleanSupplier onlyWhile, Runnable atEnd, Time time){
        return new ConditionalCommand(toSingleCoordinate(coordinate, gains, init, onlyWhile, atEnd , time),
         toSingleCoordinate(TidalUtil.coordinateFlip(coordinate), gains, init, onlyWhile, atEnd, time), isBlue());
    }

    public ConditionalCommand deadlinePathFindingCommand(Coordinate coordinate, PathConstraints gains, Runnable init, BooleanSupplier onlyWhile, Runnable atEnd, Command other){
        return new ConditionalCommand(toSingleCoordinate(coordinate, gains, init, onlyWhile, atEnd , other),
         toSingleCoordinate(TidalUtil.coordinateFlip(coordinate), gains, init, onlyWhile, atEnd, other), isBlue());
    }

    public boolean atGoal(){
        return new Grid(goal, acc).atGrid(TidalUtil.pose2dToCoordinate(drive.getPose()));
    }

    public Coordinate getGoal(){
        return goal;
    }

    public double getInaccuracy(){
        return acc;
    }

    public Coordinate lastCoordinate(){
        return TidalUtil.pose2dToCoordinate(drive.getPose());
    }

    public void stopVehicle(){
        drive.stopWithX();
    }

    @Override
    public void periodic(){

    }


}
