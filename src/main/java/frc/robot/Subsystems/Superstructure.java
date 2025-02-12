package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants.Algae;
import frc.robot.Constants.WristConstants.Coral;
import frc.robot.Subsystems.Components.AlgaeWrist;
import frc.robot.Subsystems.Components.CoralWrist;
import frc.robot.Subsystems.Components.Elevator;
import frc.robot.Subsystems.Components.Hanger;

public class Superstructure extends SubsystemBase{

    //Inicia los subsistemas
    private final Leds leds;
    public final CoralWrist wrist;
    public final AlgaeWrist algae;
    public final Elevator elevator;
    public final Hanger colgador;

    private boolean retracted = true; //Init the default value of retracted (Asume que empieza el robot retraido)

    public Superstructure(){

        leds = new Leds(); 
        wrist = new CoralWrist();
        algae = new AlgaeWrist();
        elevator = new Elevator();
        colgador = new Hanger();

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
        
        run(()->wrist.retract()). //muñeca se retrae
        onlyWhile(()-> !wrist.atGoal()).// solo si no esta en el setpoint
            beforeStarting(initStructure()). // antes checa si los motores estan apagados
            finallyDo(()->wrist.stop()).// al final para motores
                andThen( // despues
                    run(()->elevator.retract()).//retrae el elvador
                        onlyWhile(()-> !elevator.atGoal()).// solo si el elevador no esta en el setpoint
                        finallyDo(()-> { //finalmente
                            elevator.stop(); //Se detiene los motores del elevador
                            retractCompleted(); //Se cvompleta la función
                        }
                            
                        ));

        // al final escoje entre uno de los dos estados, si esta retraido se va a los leds y si no procede a retraerse
        return new ConditionalCommand(idling, retractedRequest, isStructureRetracted());
        
    }
    /**
     * Se encaraga de que el robot se cuelgue 
     * @param speed //Velocidad a la que el colgador va
     * @return // Pide que se cuelgue
     */
    public Command StructureClimb(double speed){ 
        Command climbRequest = //Comando que cuelgue el robot 
        run(()-> colgador.setSpeed(speed)). //Colgador se ejecuta a velocidad deseada
            onlyWhile(()-> !colgador.limitPressed()). //Solo mientras el colgador no ha llegado al limite 
                finallyDo( //no retrae la estructura porque ya va a estar retraida
                    ()-> colgador.stopAll()); //Se detiene los motores del colgador

        return climbRequest; //Pide que se cuelgue 
    }
    /**
     * Expulsar el coral en el nivel 4
     * @param trigger //
     * @return
     */
    public Command outakeL4(BooleanSupplier trigger){

        Command requestL4 =

        run(()->elevator.toL4()). //se va arriba
        onlyWhile(()-> !elevator.atGoal()). //mientras no haya llegado
            andThen(run(()->wrist.requestPosition(Coral.SETPOINT_OUTAKE)). //Ya llego, alinea la muñeca :v
                onlyWhile(()-> !wrist.atGoal())). //Mientras no haya llegado la muñeca
                    andThen( //ya que llego a 90 grados
                        run(()-> wrist.wheelSpeed(Coral.OUT_SPEED_L4)).
                        beforeStarting(Commands.waitSeconds(0.03). 
                            withTimeout(Coral.TIME_L4)).
                                deadlineFor(run(()->wrist.requestPosition(Coral.SETPOINT_OUT_L4)))
        );

        if (trigger.getAsBoolean()) {
           requestL4.alongWith(algaeCommand(Algae.INT_SPEED, Algae.INT_TIME_ALGAE));
        }

        requestL4.finallyDo(()-> requestRetracting());

        return requestL4;

    }

    public Command StructureProcess(){

        Command process = algaeCommand(Algae.PROCESS_SPEED, Algae.OUT_TIME_ALGAE).
        andThen(()-> algae.retract()).onlyWhile(()-> !algae.atGoal()).finallyDo(()-> algae.stop());

        return process;
    }

    public Command StructureL1(){

        Command requestL1 = 

        run(()->
            wrist.requestPosition(Coral.SETPOINT_OUT_L1)).
            beforeStarting(initStructure()).
            onlyWhile(()-> !wrist.atGoal()).
                andThen(
                    run(()-> wrist.wheelSpeed(Coral.OUT_SPEED_L1)).
                        withTimeout(0.9).
                        finallyDo(()-> { //Para la muñeca y marca que nos vamos a retraer despues 
                            wrist.wheelSpeed(0);
                            requestRetracting();
                        })
                    );

        return requestL1;
    }

    public Command StructureL2(){

        Command requestL2 = 
        
        run(()-> elevator.toL2()).onlyWhile(()-> !elevator.atGoal()).
        beforeStarting(initStructure()). 
                andThen(run(()-> wrist.requestPosition(Coral.SETPOINT_OUTAKE))). //Mueve la muñeca del coral para anotar en L2
                    onlyWhile(()-> !wrist.atGoal()).
                andThen(run(()-> wrist.wheelSpeed(Coral.OUT_SPEED_L2)).// Cuando ya esta bien posicionada, la dispara (AJUSTAR LAS CONSTANTES)
                    withTimeout(Coral.TIME_L2).
                    finallyDo(()-> {
                        wrist.wheelSpeed(0); 
                        requestRetracting();
                        })
                    );

        return requestL2;
    }

    public Command StructureL3(BooleanSupplier trigger){

        Command requestL3coral = 

        run(()->elevator.toL3()).
        beforeStarting(initStructure()).
        onlyWhile(()->!elevator.atGoal()).
            andThen(run(()->wrist.requestPosition(Coral.SETPOINT_OUTAKE)).
                onlyWhile(()->!wrist.atGoal())).
                    andThen(
                        run(()-> wrist.wheelSpeed(Coral.OUT_SPEED_L3)).
                            withTimeout(Coral.TIME_L3).
                                finallyDo(()-> requestRetracting())
                    );
        
        if (trigger.getAsBoolean()) {
            requestL3coral.alongWith(algaeCommand(Algae.INT_SPEED, Algae.INT_TIME_ALGAE));
        }   
        
        return requestL3coral;

    }

    public Command algaeCommand(double velocity, double time){

        Command algaeRequest = 
        run(()-> algae.outake()).
            onlyWhile(()-> !algae.atGoal()).
                andThen(run(()-> algae.runWheels(velocity)).
                withTimeout(time));

        algaeRequest.finallyDo(()-> algae.runWheels(0));

        return algaeRequest;

    }

    public Command algaeCommand(double velocity){

        Command algaeRequest = 
        run(()-> algae.outake()).
            onlyWhile(()-> !algae.atGoal()).
                andThen(run(()-> algae.runWheels(velocity)));

        algaeRequest.finallyDo(()-> algae.runWheels(0));

        return algaeRequest;

    }

    public Command StructureFeedTime(){
        Command requestFeed = 

        run(()-> elevator.toFeeder()).onlyWhile(()-> !elevator.atGoal()).
        beforeStarting(initStructure()).
            andThen(run(()-> wrist.requestPosition(Coral.SETPOINT_INTAKE))).
                onlyWhile(()-> !wrist.atGoal()).
                    alongWith( //Comand that moves the wheels to intake 
                        run(()-> wrist.wheelSpeed(Coral.INTAKE_SPEED)).
                        withTimeout(Coral.TIME_FEED)
        );

        //The final action to do
        requestFeed.finallyDo(()->{
            wrist.wheelSpeed(0);
            requestRetracting();
        });

        return requestFeed;
    }

    public Command StructureFeedAuto(){
        Command requestFeed = 

        run(()-> elevator.toFeeder()).onlyWhile(()-> !elevator.atGoal()).
        beforeStarting(initStructure()).
            andThen(run(()-> wrist.requestPosition(Coral.SETPOINT_INTAKE))).
                onlyWhile(()-> !wrist.atGoal()).
                    alongWith( //Comand that moves the wheels to intake 
                        run(()-> wrist.wheelSpeed(Coral.INTAKE_SPEED)).
                        onlyWhile(()-> !wrist.hasPiece())
        );

        //The final action to do
        requestFeed.finallyDo(()->{
            wrist.wheelSpeed(0);
            requestRetracting();
        });

        return requestFeed;
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
                wrist.wheelSpeed(0); //stops the motor if moving
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
    public void stopStructure(){
        wrist.stop();
        elevator.stop();
        algae.stop();
        colgador.stopAll();
    }

}
