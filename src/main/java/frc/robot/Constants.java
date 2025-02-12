package frc.robot;

import frc.robot.lib.util.Gains;
import frc.robot.lib.util.ModuleMap;

public class Constants {

    //Swerve 1-8
    //Elevador 15-16
    //Muñeca alga 12-14
    //Muñeca coral 10-11
    //Colgador 17-18

    public class ConstantsHanger {

        public static final int RightHangerPort = 17;
        public static final int LeftHangerPort = 18;
        public static final int switchPort = 0;
        public static final boolean r_inverted = true;
        public static final boolean l_inverted = false;
    }

    public class ElevatorConstants {

        //different profile PID GAINS
        public static final Gains k1_GAINS = new Gains(0.054, 0, 0.001);
        public static final Gains k2_GAINS = new Gains(0.045, 0, 0);
        public static final Gains k3_GAINS = new Gains(0.045, 0, 0);
        public static final int CAN_ID_LEADER = 15;
        public static final int CAN_ID_SLAVE = 16;
        public static final boolean leaderInverted = false;
        public static final boolean slaveInverted = true;
        public static final double IDLE_POSITION = 63;
        public static final double SETPOINT_RETRACT = IDLE_POSITION + 3;
        public static final double SETPOINT_FEEDER = 83.7;
        public static final double SETPOINT_L2 = 73.9;
        public static final double SETPOINT_L3 = 113.5;
        public static final double SETPOINT_L4 = 188;
        public static final double ERROR_TOLERANCE = 1.5; //error of 1.5 centimeters 
        public static final double CONVERSION_FACTOR = 120.9 / 20.85; 
        public static final double ELEVATOR_OFFSET = 63;
    }

    public class WristConstants {
  
        public class Algae {

            public static final int CAN_ID_WRIST = 12;
            public static final int CAN_ID_RIGHTWHEEL = 13;
            public static final int CAN_ID_LEFTWHEEL = 14;
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
            public static final double OUT_TIME_ALGAE=0; //CAMBIAR
            public static final double INT_TIME_ALGAE= 0; //CAMBIAR
            public static final double INT_SPEED = 0.7;
            public static final double THROW_SPEED = -0.7;
            public static final double PROCESS_SPEED = -0.3;

        }

        public class Coral {

            public static final int DIO_PORT_SENSOR = 0;
            public static final int CAN_ID_WRIST = 10;
            public static final int CAN_ID_EATER = 11;
            public static final boolean wristMotorInverted = false;
            public static final boolean wheelInverted = false;
            public static final int wristCurrentLimit = 40;
            public static final int wheelCurrentLimit = 15;
            public static final Gains Gains = new Gains(0,0,0);
            public static final double encoderPositionFactor = 360; //degrees
            public static final double INTAKE_SPEED = 0;
            public static final double wristErrorTolerance = 0.1; 
            public static final double TIME_L1 = 0.9; //tiempo que tarda en acabar el comando
            public static final double TIME_L2 = 1.2;
            public static final double TIME_L3 = 1.2;
            public static final double TIME_L4 = 1.2;
            public static final double TIME_OUT_L4 = 1.2;
            public static final double TIME_FEED = 0.5;
            public static final double OUT_SPEED_L1 = -0.3;
            public static final double OUT_SPEED_L2 = -0.4;
            public static final double OUT_SPEED_L3 = -0.3;
            public static final double OUT_SPEED_L4 = -0.3;
            public static final double SETPOINT_RETRACT = 0;
            public static final double SETPOINT_OUTAKE = 0; //cambiar
            public static final double SETPOINT_OUT_L4 = 0; //cambiar
            public static final double SETPOINT_OUT_L1 = 0; //cambiar
            public static final double SETPOINT_INTAKE = 0; //cambiar   
         
        }

    }
  
  public class DriveConstants {

    public static final Gains driveGains = new Gains(0.02, 0, 0, 0.06, 0.08); //0.13
    public static final Gains turnGains = new Gains(3.8, 0, 0);
    public static final Gains yGains = new Gains(0.75, 0, 0.05);

    public static final class frontLeft{

        public static final int DrivePort = 1; 
        public static final int TurnPort = 2; 
        public static final int EncPort = 1;
        public static final double offset = 268.5;                                                                                                                                                                                                                                                                                                                                                                                                                                                                        ; //48     //93  //138      //48 o 138 o 228
 
        public static final boolean DrivemotorReversed = true;
        public static final boolean TurnmotorReversed = true;

        public static final ModuleMap fl = new ModuleMap(
        DrivePort,
        TurnPort,
        EncPort,
        offset,
        DrivemotorReversed,
        TurnmotorReversed);
        
    }

    public static final class frontRight{

        public static final int DrivePort = 4; 
        public static final int TurnPort = 3; 
        public static final int EncPort = 0; 
        public static final double offset = 314.8; 
 
        public static final boolean DrivemotorReversed = false;
        public static final boolean TurnmotorReversed = true;

        public static final ModuleMap fr = new ModuleMap(
        DrivePort,
        TurnPort,
        EncPort,
        offset,
        DrivemotorReversed,
        TurnmotorReversed);

    }

    public static final class backLeft{

        public static final int DrivePort = 5; 
        public static final int TurnPort = 6; 
        public static final int EncPort = 2; 
        public static final double offset = 151.5;
 
        public static final boolean DrivemotorReversed = true;
        public static final boolean TurnmotorReversed = true; 

        public static final ModuleMap bl = new ModuleMap(
        DrivePort,
        TurnPort,
        EncPort,
        offset,
        DrivemotorReversed,
        TurnmotorReversed);

    }

    public static final class backRight{

        public static final int DrivePort = 7; 
        public static final int TurnPort = 8; 
        public static final int EncPort = 3; 
        public static final double offset = 334.5; 
 
        public static final boolean DrivemotorReversed = false;
        public static final boolean TurnmotorReversed = true;

        public static final ModuleMap br = new ModuleMap(
        DrivePort,
        TurnPort,
        EncPort,
        offset,
        DrivemotorReversed,
        TurnmotorReversed);

    }

}

}
