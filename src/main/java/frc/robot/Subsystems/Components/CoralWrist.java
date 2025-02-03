package frc.robot.Subsystems.Components;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants.Coral;

public class CoralWrist extends SubsystemBase{
    
    public SparkMax wrist; 
    public SparkMax eater;
    public AbsoluteEncoder throughBore;
    public SparkClosedLoopController controller;

    public SparkMaxConfig ConfigWrist , ConfigEater;

    public boolean atGoal;

    public double target;

    public CoralWrist(){

        atGoal = false;

        wrist = new SparkMax(Coral.CAN_ID_WRIST, MotorType.kBrushless);

        eater = new SparkMax(Coral.CAN_ID_EATER, MotorType.kBrushless);

        throughBore = wrist.getAbsoluteEncoder();

        controller = wrist.getClosedLoopController();

        ConfigWrist = new SparkMaxConfig();
        ConfigEater = new SparkMaxConfig();

        Burnflash();
    }

    private void Burnflash(){

        wrist.setCANTimeout(250);
        eater.setCANTimeout(250);

        //Config wrist
        ConfigWrist.absoluteEncoder.
            inverted(Coral.throughBoreInverted).
            positionConversionFactor(Coral.encoderPositionFactor);

        ConfigWrist.
            inverted(Coral.wristMotorInverted).
            idleMode(IdleMode.kBrake).
            smartCurrentLimit(Coral.wristCurrentLimit).
        closedLoop.
            p(Coral.closedLoopPID.getP()).
            i(Coral.closedLoopPID.getI()).
            d(Coral.closedLoopPID.getD()).
            feedbackSensor(FeedbackSensor.kAbsoluteEncoder).
        maxMotion.
            maxVelocity(Coral.MAX_VELOCITY).
            maxAcceleration(Coral.MAX_ACC).
            positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

        //Config eater
        ConfigEater.
            inverted(Coral.eaterInverted).
            idleMode(IdleMode.kCoast).
            smartCurrentLimit(Coral.eaterCurrentLimit);

        wrist.configure(ConfigWrist, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        eater.configure(ConfigEater, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        wrist.setCANTimeout(0);
        eater.setCANTimeout(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("[CORALWRIST]: Position:", reportPosition());
        SmartDashboard.putBoolean("[CORALWRIST]: AtGoal:"  , atGoal);
    }
    public double reportPosition(){
        return throughBore.getPosition();
    }
    public void requestPosition(double degrees){
        controller.setReference(degrees,ControlType.kMAXMotionPositionControl);
    }
    public void requestEater(double speed){
        eater.set(speed);
    }
    public void retract(){
        requestPosition(0);
    }

    public void lockPosition(){
        requestPosition(reportPosition());
    }

    public void stop(){
        wrist.stopMotor();
        eater.stopMotor();
    }
    public boolean atGoal(){
        return Math.abs(reportPosition() - target) <= Coral.wristErrorTolerance;
    }

}
