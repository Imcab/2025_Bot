package frc.robot.lib;

import frc.robot.lib.util.Elastic;
import frc.robot.lib.util.Elastic.Notification;
import frc.robot.lib.util.Elastic.Notification.NotificationLevel;

public class Alerts {

    //Storage for all the notifications through elastic dashboard

    public static Notification lowBattery =
         new Notification(NotificationLevel.ERROR, "Minimum Battery detected!", "Change the battery NOW");
    public static Notification mediumBattery =
         new Notification(NotificationLevel.WARNING, "Low Battery detected!", "Change the battery");
        
    public static Notification navX_Reset =
         new Notification(NotificationLevel.INFO, "A navX reset has ocurred", "the robot's heading has been changed");
    public static Notification navX_Disconnected = 
         new Notification(NotificationLevel.ERROR, "NAVX DISCONNECTED!", "Using alternate heading method");
    public static Notification navX_FastSpeed =
         new Notification(NotificationLevel.WARNING, "Angular speed is over limit!", "any vision updates will be ignored");
    public static Notification navX_Connected =
         new Notification(NotificationLevel.INFO, "navX is now connected", "all settings working perfectly!");
        
    public static Notification OK = 
         new Notification(NotificationLevel.INFO, "No problems detected!","Systems working perfectly");
        
    public static Notification encoderShuts =
         new Notification(NotificationLevel.WARNING, "Disconnecting a swerve encoder...", "Now using internal encoder");
    public static Notification holdingAngle =
         new Notification(NotificationLevel.INFO, "Locking Swerve to 0 degrees", "Release the button to unlock");
        
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
    public static void sendNavxConnected(){
        Elastic.sendNotification(navX_Connected);
    }
    public static void sendFastSpeed(){
        Elastic.sendNotification(navX_FastSpeed);
    }
    public static void sendOK(){
        Elastic.sendNotification(OK);
    }
    public static void sendEncoderShuts(){
        Elastic.sendNotification(encoderShuts);
    }
    public static void sendHold(){
        Elastic.sendNotification(holdingAngle);
    }
    public static void sendMediumBattery(){
        Elastic.sendNotification(mediumBattery);
    }

}
