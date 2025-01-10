//v1.0.2
package frc.robot.lib.util;

/**
 * Represents PID and motion parameters for control systems.
 */
public class Gains {
    
    private double kP,kI,kD,kS,kV, acc, jerk;

    private double lastkP, lastkI, lastkD, lastkS, lastkV, lastAcc, lastJerk;

    public Gains(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = 0;
        this.kS = 0;
        this.acc = 0;
        this.jerk = 0;
    }

    public Gains(double kP, double kI, double kD, double kS, double kV){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.kS = kS;
        this.acc = 0;
        this.jerk = 0;
    }

    public Gains(double kP, double kI, double kD, double kS, double kV, double acc, double jerk){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.kS = kS;
        this.acc = acc;
        this.jerk = jerk;
    }
    
    //Getters
    public double getAcceleration(){
        return acc;
    }
    public double getJerk(){
        return jerk;
    }
    public double getP(){
        return kP;
    }
    public double getI(){
        return kI;
    }
    public double getD(){
        return kD;
    }
    public double getS(){
        return kS;
    }
    public double getV(){
        return kV;
    }
    //Setters
    public void setAcceleration(double value){
        this.acc = value;
    }
    public void setJerk(double value){
        this.jerk = value;
    }
    public void setP(double value){
        this.kP = value;
    }
    public void setI(double value){
        this.kI = value;
    }
    public void setD(double value){
        this.kD = value;
    }
    public void setS(double value){
        this.kS = value;
    }
    public void setV(double value){
        this.kV = value;
    }

    //Checks if a value has changed
    public boolean hasChanged() {
        boolean changed = !(
            kP == lastkP &&
            kI == lastkI &&
            kD == lastkD &&
            kS == lastkS &&
            kV == lastkV &&
            acc == lastAcc &&
            jerk == lastJerk
        );

        //Update to the last values
        if (changed) {
            lastkP = kP;
            lastkI = kI;
            lastkD = kD;
            lastkS = kS;
            lastkV = kV;
            lastAcc = acc;
            lastJerk = jerk;
        }

        return changed;
    }

    //Resets all the values to 0
    public void reset(){
        setAcceleration(0);
        setJerk(0);
        setP(0);
        setI(0);
        setD(0);
        setS(0);
        setV(0);

        lastkP = 0;
        lastkI = 0;
        lastkD = 0;
        lastkS = 0;
        lastkV = 0;
        lastAcc = 0;
        lastJerk = 0;
    }
}
