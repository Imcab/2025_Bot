package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants.Coral;
import frc.robot.Subsystems.Components.AlgaeWrist;
import frc.robot.Subsystems.Components.CoralWrist;
import frc.robot.Subsystems.Components.Elevator;
import frc.robot.Subsystems.Components.HangerKraken;

public class Superstructure extends SubsystemBase{

    //Inicia los subsistemas
    private final Leds leds;
    private final CoralWrist wrist;
    private final AlgaeWrist algae;
    private final Elevator elevator;
    private final HangerKraken colgador;

    private boolean retracted = true; //Init the default value of retracted (Asume que empieza el robot retraido)

    public Superstructure(){

        leds = new Leds(); 
        wrist = new CoralWrist();
        algae = new AlgaeWrist();
        elevator = new Elevator();
        colgador = new HangerKraken();

        //Pone como default commando el retract, este comando corre aunque estemos en disabled por los leds
        setDefaultCommand(StructureRetract().ignoringDisable(true)); 

    }

    @Override
    public void periodic(){
        leds.periodic(); //Update the leds
        SmartDashboard.putString("[SUPERSTRUCTURE] CURRENT COMMAND: ", getCurrentCommand() != null ? getCurrentCommand().getName() : "NONE");
    }

    /**
     * Checa si un comando de la super estructura ha terminado
     * @param request el comando de la super estructura
     * @return si ha terminado o no!
     */
    public boolean checkIfFinished(Command request){
        return request.isFinished();
    }

    /**
     * Checa si un comando de la super estructura esta activo
     * @param request el comando a checar
     * @return si esta activo o ne
     */
    public boolean isActive(Command request){
        return request.isScheduled();
    }

    /**
     * Regresa el comando actual de la SP
     * @return el comando
     */
    public Command currentCommand(){
        return getCurrentCommand(); //returns the current Command
    }

    /**
     * Se encarga de manejar el estado default de la superEstructura
     * @return se retrae o se queda quieto dependiendo de si ya se ha retraido o no
     */
    public ConditionalCommand StructureRetract(){

        //Comando default, cuando ya estemos retraidos, no hacer nada o poner cosas guays, nose (efecto de leds disabled)
        Command idling = 

        run(()->{
            //leds.robotDisabled();
        });

        //Commando de Retraer la estructura
        Command retractedRequest =

        run(()->wrist.retract()).
        onlyWhile(()-> !wrist.atGoal()).
            beforeStarting(initStructure()). 
            finallyDo(()->wrist.stop()).
                andThen(
                    run(()->elevator.retract()).
                        onlyWhile(()-> !elevator.atGoal()).
                        finallyDo(()-> {
                            elevator.stop();
                            retractCompleted();
                        }
                            
                        ));

        // al final escoje entre uno de los dos estados, si esta retraido se va a los leds y si no procede a retraerse
        return new ConditionalCommand(idling, retractedRequest, isStructureRetracted());
        
    }

    public Command StructureL1(){

        Command requestL1 = 

        run(()->
            wrist.requestPosition(Coral.L1)).
            beforeStarting(initStructure()).
            onlyWhile(()-> !wrist.atGoal()).
                andThen(
                    run(()-> wrist.requestEater(Coral.SPEED_L1)).
                        withTimeout(0.9).
                        finallyDo(()-> { //Para la muñeca y marca que nos vamos a retraer despues 
                            wrist.stop();
                            requestRetracting();
                        })
                    );

        return requestL1;
    }

    private void retractCompleted(){
        this.retracted = true; //pone el retracted en true, USAR AL COMPLETAR UN RETRACT SOLAMENTE
    }

    public void requestRetracting(){
        this.retracted = false; //Pone el retracted en false, lo que significa que se debe de retraer al finalizar este comando
    }

    private Command initStructure(){

        //Checa si hay movimiento y si si, lo apaga
        return runOnce(()->{
            
            if (wrist.isWheelSpinning()) {
                wrist.requestEater(0); //stops the motor if moving
            }

            if (algae.areWheelsSpinning()) {
                algae.runWheels(0); //stops the motors if moving
            }

        });
    }

    /**
     * Checks if the superStructure is retracted
     * @return if it is retracted
     */
    public BooleanSupplier isStructureRetracted(){
        return ()-> retracted; 
    }

    /**
     * Para todos nuestros subsistemas
     */
    public void stop(){
        wrist.stop();
        elevator.stop();
        algae.stop();
        colgador.stopAll();
    }

}
