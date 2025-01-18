package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.lib.SwerveConfig;
import frc.robot.lib.vision.LimelightHelpers;
import frc.robot.lib.vision.VisionConfig.limelight;

public class RobotState{

    //possible states from battery
    public enum batteryCharge{
        mLOW, mMEDIUM, mHIGH
    }

    //init default values
    public static double batteryVoltsTrigger = 7.5;
    public static double batteryVoltsTriggerMed = 10;
    public static double gyroRate;
    public static boolean isEndgame = false;
    public static final double mPeriod = 0.02;
    public static boolean mConnection = false;
    public static batteryCharge mCharge;
    public static final String NetworkTables_AlertGroupID = "SystemState";
    
    //mutators and access methods

    public static batteryCharge batteryState(){
        if (RobotController.getBatteryVoltage() < batteryVoltsTrigger) {
            mCharge = batteryCharge.mLOW;
        }else 
        if (RobotController.getBatteryVoltage() < batteryVoltsTriggerMed && RobotController.getBatteryVoltage() > batteryVoltsTrigger) {
            mCharge = batteryCharge.mMEDIUM;
        }else 
        if (RobotController.getBatteryVoltage() > batteryVoltsTriggerMed) {
            mCharge = batteryCharge.mHIGH;
        }
        return mCharge;
    }

    public static boolean limeHasTarget(){
        return LimelightHelpers.getTV(limelight.name);
    }

    public static double getMatchTime(){
        return DriverStation.getMatchTime();
    }

    public void endGameStart(boolean end){
        isEndgame = end;
    }

    public static boolean isEndgame(){
        return isEndgame;
    }
    public static void setAngularVelocity(double Rate){
        gyroRate = Rate;
    }

    public static double getAngularVelocity(){
        return gyroRate;
    }

    public static boolean isRed(){
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red);
    }

    public static boolean isAngularVelAboveLimits(){
        return Math.abs(gyroRate) > SwerveConfig.gyro.angularSpeedTrigger;
    }

    public static boolean getGyroConnection(){
        return mConnection;
    }

    public static void setGyroConnection(boolean value){
        mConnection = value;
    }

}
