package frc.robot.lib;

import frc.robot.lib.util.Elastic;
import frc.robot.lib.util.Elastic.Notification;
import frc.robot.lib.util.Elastic.Notification.NotificationLevel;

public class Alerts {

    //Storage for all the notifications through elastic dashboard

    public static final double mHeight = 12; //Notification height displayed on Elastic

    public static final double defaulWith = 350;

    public static final double extendedWith = 450;

    public static final double infoSeconds = 2.0;

    public static final double warningSeconds = 4.0;

    public static final double errorSeconds = 7.0;

    public static Notification lowBattery =
         new Notification(NotificationLevel.ERROR, "Low Battery detected!", "Change the battery NOW", defaulWith,mHeight).withDisplaySeconds(errorSeconds);
    public static Notification mediumBattery =
         new Notification(NotificationLevel.WARNING, "Approaching low battery!", "Change the battery", defaulWith, mHeight).withDisplaySeconds(warningSeconds);
        
    public static Notification navX_Reset =
         new Notification(NotificationLevel.INFO, "A navX reset has ocurred", "the robot's heading has been changed", defaulWith, mHeight).withDisplaySeconds(infoSeconds);
    public static Notification navX_Disconnected = 
         new Notification(NotificationLevel.ERROR, "NAVX DISCONNECTED!", "Using alternate heading method", defaulWith, mHeight).withDisplaySeconds(errorSeconds);
    public static Notification navX_FastSpeed =
         new Notification(NotificationLevel.WARNING, "Angular speed is over limit!", "any vision updates will be ignored", defaulWith, mHeight).withDisplaySeconds(warningSeconds);      
    public static Notification encoderShuts =
         new Notification(NotificationLevel.WARNING, "Disconnecting a swerve encoder...", "Now using internal encoder", defaulWith,mHeight).withDisplaySeconds(warningSeconds);

    private Alerts(){}
    
    public static void sendCustom(Notification notification){
            Elastic.sendNotification(notification);
    }

        public static void sendLowBattery(){
            Elastic.sendNotification(lowBattery);
    }
    public static void sendNavxReset(){
        Elastic.sendNotification(navX_Reset);
    }
    public static void sendNavxDisconnected(){
        Elastic.sendNotification(navX_Disconnected);
    }
    public static void sendFastSpeed(){
        Elastic.sendNotification(navX_FastSpeed);
    }
    public static void sendEncoderShuts(){
        Elastic.sendNotification(encoderShuts);
    }
    public static void sendMediumBattery(){
        Elastic.sendNotification(mediumBattery);
    }

}
