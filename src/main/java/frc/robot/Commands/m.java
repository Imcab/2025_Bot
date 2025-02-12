package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Components.Elevator;

public class m extends Command{
    Elevator e;
    DoubleSupplier x;
    public m(Elevator e, DoubleSupplier x){
        this.e = e;
        this.x = x;
        addRequirements(e);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute(){
        double c = x.getAsDouble();
        e.runSpeed(c);
    }

    @Override
    public boolean isFinished() {
        return e.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        e.stop();
    }
}
