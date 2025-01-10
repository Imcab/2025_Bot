package frc.robot;

import edu.wpi.first.wpilibj.RobotController;

public class RobotState{

    public static double batteryVoltsTrigger = 7.5;
    public static double batteryVoltsTriggerMed = 10;
    public static double gyroRate;
    public static boolean isEndgame = false;
    public static boolean below12VoltsInit = false;
    public static double mPeriod = 0.0;

    public static boolean isLowBattery(){
        return RobotController.getBatteryVoltage() <= batteryVoltsTrigger;
    }

    public static void setPeriod(double p){
        mPeriod = p;
    }

    public static double getPeriod(){
        return mPeriod;
    }
    public static boolean isMediumCharge(){
        return RobotController.getBatteryVoltage() <= batteryVoltsTriggerMed;
    }

    public void endGameStart(boolean end){
        isEndgame = end;
    }

    public static boolean isEndgame(){
        return isEndgame;
    }

    public static void startwithLowMode(boolean toggle){
        below12VoltsInit =toggle;
    }
    public static boolean isLowModeSetted(){
        return below12VoltsInit;
    }

    public static void setAngularVelocity(double Rate){
        gyroRate = Rate;
    }

    public static double getAngularVelocity(){
        return gyroRate;
    }

    public static boolean isAngularVelAboveLimits(){
        return gyroRate > 720;
    }
}
