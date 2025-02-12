package frc.robot.Subsystems.Components;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants.Coral;
import frc.robot.lib.util.BeamSensor;

public class CoralWrist extends SubsystemBase{
    
    private SparkMax wrist; 
    private RelativeEncoder wristEncoder;
    private SparkMax eater;

    private SparkClosedLoopController controller;

    private SparkMaxConfig ConfigWrist , ConfigEater;

    private boolean atGoal;

    private double target;

    private BeamSensor beamBreaker = new BeamSensor(Coral.DIO_PORT_SENSOR); 

    //Declaracion de 
    public CoralWrist(){

        atGoal = false;

        wrist = new SparkMax(Coral.CAN_ID_WRIST, MotorType.kBrushless);

        wristEncoder = wrist.getEncoder();

        controller = wrist.getClosedLoopController();

        eater = new SparkMax(Coral.CAN_ID_EATER, MotorType.kBrushless);

        ConfigWrist = new SparkMaxConfig();
        ConfigEater = new SparkMaxConfig();

        Burnflash();

        setZero();
    }

    private void Burnflash(){

        wrist.setCANTimeout(250);
        eater.setCANTimeout(250);

        ConfigWrist.
            inverted(Coral.wristMotorInverted).
            idleMode(IdleMode.kBrake).
            smartCurrentLimit(Coral.wristCurrentLimit).
        closedLoop.
            p(Coral.Gains.getP()).
            i(Coral.Gains.getI()).
            d(Coral.Gains.getD()).
            feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        //Config eater
        ConfigEater.
            inverted(Coral.wheelInverted).
            idleMode(IdleMode.kCoast).
            smartCurrentLimit(Coral.wheelCurrentLimit);

        wrist.configure(ConfigWrist, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        eater.configure(ConfigEater, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        wrist.setCANTimeout(0);
        eater.setCANTimeout(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("[CORALWRIST]: Position:", getPosition());
        SmartDashboard.putNumber("[CORALWRIST]: RawPosition:", getRawPosition());
        SmartDashboard.putBoolean("[CORALWRIST]: AtGoal:"  , atGoal);
        SmartDashboard.putNumber("[CORALWRIST]: Target:", target);
    }
    
    
    public boolean hasPiece(){
        return beamBreaker.get();
    }

    public double getRawPosition(){
        return wristEncoder.getPosition();
    }
    public double getPosition(){
        return Degrees.convertFrom(getRawPosition(), Rotations);
    }

    public void setZero(){
        wristEncoder.setPosition(0);
    }

    public void requestPosition(double degrees){
        this.target = degrees;
        controller.setReference(Units.degreesToRotations(degrees),ControlType.kPosition);
    }

    public void wheelSpeed(double speed){
        eater.set(speed);
    }
    public void retract(){
        requestPosition(0);
    }

    public boolean isWheelSpinning(){
        return eater.get() != 0;
    }

    public void lockPosition(){
        requestPosition(getRawPosition());
    }

    public void stop(){
        wrist.stopMotor();
        eater.stopMotor();
    }

    public boolean atGoal(){
        return Math.abs(getRawPosition() - target) <= Coral.wristErrorTolerance;
    }

}
