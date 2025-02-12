package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Components.Elevator;

public class k extends Command{
    Elevator e;
    double y;
    public k(Elevator e, double y){
        this.e = e;
        this.y = y;
        addRequirements(e);
    }

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute(){
        e.runRequest(y);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        e.stop();
    }
}
