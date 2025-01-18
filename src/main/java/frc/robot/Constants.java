package frc.robot;


import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.lib.util.Gains;

public class Constants {

  public class ElevatorConstants {

        public static final int CAN_ID_LEADER = 0;
        public static final int CAN_ID_SLAVE = 0;
        public static final InvertedValue leaderIV = InvertedValue.Clockwise_Positive;
        public static final InvertedValue slaveIV = InvertedValue.Clockwise_Positive;
        public static final double elevator_reduction = 0;
        public static final Gains motionMagicGains = new Gains(
            0,
            0,
            0,
            0,
            0,
            0,
            0
            );
        public static final double SETPOINT_RETRACT = 0;
        public static final double SETPOINT_L1 = 0;
        public static final double SETPOINT_L2 = 0;
        public static final double SETPOINT_L3 = 0;
        public static final double SETPOINT_L4 = 0;
        public static final double MM_Acc = 0;
        public static final double MM_Jerk = 0;
        public static final double MM_CruiseVel = 0;
        public static final double MM_ExpoKv = 0;
        public static final double MM_ExpoKa = 0;
        public static final double ERROR_TOLERANCE = 0;
    
  }

  public class WristConstants {
  
    public class Algae {
    
        
    }

    public class Coral {

        public static final int CAN_ID_WRIST = 0;
        public static final int CAN_ID_EATER = 0;
        public static final boolean wristMotorInverted = false;
        public static final boolean eaterInverted = false;
        public static final boolean throughBoreInverted = false;
        public static final int wristCurrentLimit = 40;
        public static final int eaterCurrentLimit = 15;
        public static final Gains closedLoopPID = new Gains(0,0,0);
        public static final double MAX_VELOCITY = 0;
        public static final double MAX_ACC = 0;
        public static final double encoderPositionFactor = 360; //degrees
        public static final double Pos_Retract = 0;
        public static final double Pos_Idle = 0;
        public static final double Pos_Intake = 0;
        public static final double wristErrorTolerance = 0.1; 
    
    }
  }
  
  public class DriveConstants {

    public static final Gains driveGains = new Gains(0.02, 0, 0, 0.1, 0.13);
    public static final Gains turnGains = new Gains(6, 0, 0);
    public static final Gains yGains = new Gains(0.75, 0, 0.05);

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
