package frc.robot;

import frc.robot.lib.util.Gains;

public class Constants {

    public class ConstantsHanger {

        public static final int RightHangerPort = 0;
        public static final int LeftHangerPort = 0;
        public static final double kP = 0.5;
        public static final double kI = 0.5;
        public static final double kD = 0.01;
    }

    public class ElevatorConstants {

        //different profile PID GAINS
        public static final Gains k1_GAINS = new Gains(0.014, 0, 0);
        public static final Gains k2_GAINS = new Gains(0.012, 0, 0);
        public static final Gains k3_GAINS = new Gains(0.009, 0, 0);
        public static final int CAN_ID_LEADER = 15;
        public static final int CAN_ID_SLAVE = 16;
        public static final boolean leaderInverted = false;
        public static final boolean slaveInverted = true;
        public static final double IDLE_POSITION = 0;
        public static final double SETPOINT_RETRACT = IDLE_POSITION + 3;
        public static final double SETPOINT_FEEDER = 83.7;
        public static final double SETPOINT_L2 = 73.9;
        public static final double SETPOINT_L3 = 113.5;
        public static final double SETPOINT_L4 = 183.5;
        public static final double ERROR_TOLERANCE = 1.5; //error of 1.5 centimeters 
        public static final double CONVERSION_FACTOR = 120.9 / 20.85; 
        public static final double ELEVATOR_OFFSET = 63;
    }

    public class WristConstants {
  
        public class Algae {

            public static final int CAN_ID_WRIST = 0;
            public static final int CAN_ID_RIGHTWHEEL = 0;
            public static final int CAN_ID_LEFTWHEEL = 0;
            public static final boolean wristMotorInverted = false;
            public static final boolean RightInverted = false;
            public static final boolean LeftInverted = false;
            public static final boolean throughBoreInverted = false;
            public static final int wristCurrentLimit = 30;
            public static final int WheelsCurrentLimit = 15;
            public static final Gains closedLoopPID = new Gains(0,0,0);
            public static final double encoderPositionFactor = 360; //degrees
            public static final double wristErrorTolerance = 0.1;
            public static final double lookDownSetpoint = 0; //0 degrees for default
            public static final double extendSetpoimt = 0; //falta por configurar

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
            public static final double LAYDOWN = 0;
            public static final double LAYDOWN_EJECT = LAYDOWN - 3; //degrees
            public static final double INTAKE = 0;
            public static final double wristErrorTolerance = 0.1; 
    
        }

    }
  
  public class DriveConstants {

    public static final Gains driveGains = new Gains(0.02, 0, 0, 0.1, 0.13);
    public static final Gains turnGains = new Gains(6, 0, 0);
    public static final Gains yGains = new Gains(0.75, 0, 0.05);

    public static final class frontLeft{

        public static final int DrivePort = 1; 
        public static final int TurnPort = 2; 
        public static final int EncPort = 0;
        public static final double offset = 269.55;                                                                                                                                                                                                                                                                                                                                                                                                                                                                        ; //48     //93  //138      //48 o 138 o 228
 
        public static final boolean DrivemotorReversed = false;
        public static final boolean TurnmotorReversed = true;
        
    }

    public static final class frontRight{

        public static final int DrivePort = 4; 
        public static final int TurnPort = 3; 
        public static final int EncPort = 3; 
        public static final double offset = 316; 
 
        public static final boolean DrivemotorReversed = true;
        public static final boolean TurnmotorReversed = true;

    }

    public static final class backLeft{

        public static final int DrivePort = 5; 
        public static final int TurnPort = 6; 
        public static final int EncPort = 1; 
        public static final double offset = 323.5;
 
        public static final boolean DrivemotorReversed = true;
        public static final boolean TurnmotorReversed = true; 

    }

    public static final class backRight{

        public static final int DrivePort = 7; 
        public static final int TurnPort = 8; 
        public static final int EncPort = 2; 
        public static final double offset = 162.5; 
 
        public static final boolean DrivemotorReversed = false;
        public static final boolean TurnmotorReversed = true;

    }

}

}
