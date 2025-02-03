package frc.robot.Subsystems.Components;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants.Algae;

public class AlgaeWrist extends SubsystemBase{

    private SparkMax rightWheel, leftWheel, wrist;
    private SparkClosedLoopController wristPID; //en vez de usar directamente la clase de PID, usa el PID interno del motor
    private SparkAbsoluteEncoder encoder;
    private SparkBaseConfig configRightWheel, configLeftWheel, configWrist;
    private double Setpoint = 0; //init default (representa el setpoint actual del PID)

    public AlgaeWrist(){

        //Declarations of the 3 motors (Both sides and the Angler), the PID to get the right degree and the absolute encoder
        rightWheel = new SparkMax(Algae.CAN_ID_RIGHTWHEEL, MotorType.kBrushless);
        leftWheel = new SparkMax(Algae.CAN_ID_LEFTWHEEL, MotorType.kBrushless);
        wrist = new SparkMax(Algae.CAN_ID_WRIST, MotorType.kBrushless);

        encoder = wrist.getAbsoluteEncoder();

        //Closed loop es lo mismo q pid
        wristPID = wrist.getClosedLoopController();

        burnflash();
    }

    private void burnflash(){
        //method to configure the motors

        wrist.setCANTimeout(250);
        leftWheel.setCANTimeout(250);
        rightWheel.setCANTimeout(250);
    
        configWrist.
            inverted(Algae.wristMotorInverted).
            idleMode(IdleMode.kBrake).
            smartCurrentLimit(Algae.wristCurrentLimit).
            closedLoop.
                feedbackSensor(FeedbackSensor.kAbsoluteEncoder).
                pid(
                Algae.closedLoopPID.getP(),
                Algae.closedLoopPID.getI(),
                Algae.closedLoopPID.getD());
        configWrist.
            absoluteEncoder.
                inverted(Algae.throughBoreInverted).
                positionConversionFactor(Algae.encoderPositionFactor);

        configLeftWheel.
            inverted(Algae.LeftInverted).
            idleMode(IdleMode.kCoast).
            smartCurrentLimit(Algae.WheelsCurrentLimit);

        configRightWheel.
            inverted(Algae.RightInverted).
            idleMode(IdleMode.kCoast).
            smartCurrentLimit(Algae.WheelsCurrentLimit);
        
        wrist.configure(configWrist, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftWheel.configure(configLeftWheel, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightWheel.configure(configRightWheel, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        wrist.setCANTimeout(0);
        leftWheel.setCANTimeout(0);
        rightWheel.setCANTimeout(0);

    }

    public void extendWrist(){
        //extend the wrist to catch the algae, pone la muñeca en la posición para agarrar la pelota
        Setpoint = Algae.extendSetpoimt;
        wristPID.setReference(Setpoint, ControlType.kPosition);
    }

    public void lookDown(){
        //set the wrist to the 0 position, mirando hacia abajo
        Setpoint = Algae.lookDownSetpoint;
        wristPID.setReference(Setpoint, ControlType.kPosition);
    }

    public double getPosition(){
        //returns the absolute encoder position (throughbore) in degrees 
        return encoder.getPosition();
    }

    public double currentSetpoint(){
        //returns the current PID setpoint
        return Setpoint;
    }

    public boolean atGoal(){
        //checks if the wrist has completed the pid request
        return Math.abs(getPosition() - Setpoint) <= Algae.wristErrorTolerance;
    }

    public void runWheels(double velocity) {
        rightWheel.set(velocity);
        leftWheel.set(velocity);
        //Set the motors to the desired velocity
    }
    
    public void stop(){
        rightWheel.stopMotor();
        leftWheel.stopMotor();
        wrist.stopMotor();
        //Putting a stop to the motors
    }
}
