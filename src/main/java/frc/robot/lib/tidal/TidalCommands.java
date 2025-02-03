package frc.robot.lib.tidal;

import java.security.PublicKey;
import java.util.function.BooleanSupplier;
import java.util.function.IntFunction;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Drive.swerve;
import frc.robot.lib.tidal.TidalConfig.reef;

public class TidalCommands extends SubsystemBase{

    private TidalFinder tidal;
    private swerve drive;
    private BooleanSupplier sup;

    public TidalCommands(swerve drive , TidalFinder tidal){
        this.tidal = tidal;
        this.drive = drive;
    }

    public void setAlliance(BooleanSupplier sup){
        this.sup = sup;
    }

    public Command toLeftFeeder(){
        return Commands.runOnce(()->{if (sup.getAsBoolean()) {
            tidal.toLeftFeederBlue().schedule();
        }else{
            tidal.toLeftFeederRed().schedule();
        }}, drive);
    }

    public ConditionalCommand toRightFeeder(){
        return new ConditionalCommand(tidal.toRightFeederBlue(), tidal.toRightFeederRed(), sup);
    }
  
    public ConditionalCommand toNearest(){
        return new ConditionalCommand(toLeftFeeder(), toRightFeeder(), ()-> tidal.atLeftFeeder());
    }

    public boolean atReef(){

        if (!TidalUtil.isBlue()) {
            return reef.RedReef.atGrid(TidalUtil.pose2dToCoordinate(drive.getPose()));
        }
        return reef.BlueReef.atGrid(TidalUtil.pose2dToCoordinate(drive.getPose()));
    }

    @Override
    public void periodic(){

    }

}
