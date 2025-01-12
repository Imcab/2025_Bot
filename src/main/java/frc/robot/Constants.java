package frc.robot;

import frc.robot.lib.util.Gains;

public class Constants {
  
  public class DriveConstants {

    public static final Gains driveGains = new Gains(0.05, 0, 0, 0.1, 0.13);
    public static final Gains turnGains = new Gains(6.8, 0, 0);

    public static final double VisionRangekp = 0.0;
    public static final double VisionAimkp = 0.0;
    
    public static final class frontLeft{

        public static final int DrivePort = 6; 
        public static final int TurnPort = 5; 
        public static final int EncPort = 3;
        public static final double offset = 46;                                                                                                                                                                                                                                                                                                                                                                                                                                                                        ; //48     //93  //138      //48 o 138 o 228
 
        public static final boolean DrivemotorReversed = false;
        public static final boolean TurnmotorReversed = true;
        
    }

    public static final class frontRight{

        public static final int DrivePort = 8; 
        public static final int TurnPort = 7; 
        public static final int EncPort = 2; 
        public static final double offset = 69.6; 
 
        public static final boolean DrivemotorReversed = false;
        public static final boolean TurnmotorReversed = true;

    }

    public static final class backLeft{

        public static final int DrivePort = 2; 
        public static final int TurnPort = 1; 
        public static final int EncPort = 0; 
        public static final double offset = 0;
 
        public static final boolean DrivemotorReversed = true;
        public static final boolean TurnmotorReversed = true; 

    }

    public static final class backRight{

        public static final int DrivePort = 4; 
        public static final int TurnPort = 3; 
        public static final int EncPort = 1; 
        public static final double offset = 60.2; 
 
        public static final boolean DrivemotorReversed = false;
        public static final boolean TurnmotorReversed = true;


    }
}

}
