package frc.robot.lib.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

/**
 * Libreria de sensores conectados como Digital Input
 */
public class Sensors {
    
    /**
    * Métodos para el sensor infrarojo
    */
    public class BeamSensor{
        private final DigitalInput mbeam;
        private boolean lastStatus;
	    private boolean tripped;
	    private boolean cleared;

        private Debouncer timer;
        private double delay = 0;

        private double failureTimeoutSecs = 15.0; 
        private double lastChangeTime;
        private boolean sensorFault; 

        private final Map<Double, Debouncer> debouncerMap;

        public BeamSensor(int port){
            mbeam = new DigitalInput(port);
            timer = new Debouncer(getDelayTime());
            lastChangeTime = Timer.getFPGATimestamp();
            sensorFault = false;  
            debouncerMap = new HashMap<>();     
        }

        public void configureDelayTime(double value){
            delay = value;
        }
        public void configureFailureTimeout(double timeout) {
            this.failureTimeoutSecs = timeout;
        }

        public double getFailureTimeout(){
            return failureTimeoutSecs;
        }

        public double getDelayTime(){
            return delay;
        }

        public void update() {
            boolean value = get();
            

            if (value != lastStatus) {
                lastChangeTime = Timer.getFPGATimestamp(); 
                sensorFault = false;
            }

            double currentTime = Timer.getFPGATimestamp();
            if ((currentTime - lastChangeTime) > failureTimeoutSecs) {
                sensorFault = true; // Detecta una falla si no ha cambiado en el tiempo default
            }

            tripped = value && !lastStatus;
            cleared = !value && lastStatus;
            lastStatus = value;
        }
    
        public boolean get() {
            return !mbeam.get();
        }

        public boolean getDelayedFromSeconds(double seconds){
            debouncerMap.putIfAbsent(seconds, new Debouncer(seconds));
            return debouncerMap.get(seconds).calculate(get());
        }

        public boolean getDelayed(){
            return timer.calculate(get());
        }
    
        public boolean wasTripped() {
            return tripped;
        }

        public boolean wasCleared() {
            return cleared;
        }

        public boolean hasFaults(){
            return sensorFault;
        }
    
   } 
}
