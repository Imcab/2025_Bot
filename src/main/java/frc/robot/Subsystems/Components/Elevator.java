// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Components;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.lib.tidal.Domain;

public class Elevator extends SubsystemBase{

  private final SparkMax leader, follower;
  private final SparkMaxConfig leaderConfig, followerConfig;
  private final RelativeEncoder leaderEncoder, followerEncoder;
  private double currentSetpoint = 0; //init current setpoint

  public enum ELEVATOR_LEVELS{
    k1,k2,k3
  }

  ELEVATOR_LEVELS tracker = ELEVATOR_LEVELS.k1;

  PIDController k1Controller;
  PIDController k2Controller;
  PIDController k3Controller;

  public Elevator(){

    k1Controller = new PIDController(
    ElevatorConstants.k1_GAINS.getP(),
    ElevatorConstants.k1_GAINS.getI(),
    ElevatorConstants.k1_GAINS.getD());


    k2Controller = new PIDController(
    ElevatorConstants.k2_GAINS.getP(),
    ElevatorConstants.k2_GAINS.getI(),
    ElevatorConstants.k2_GAINS.getD());

    k3Controller = new PIDController(
    ElevatorConstants.k3_GAINS.getP(),
    ElevatorConstants.k3_GAINS.getI(),
    ElevatorConstants.k3_GAINS.getD());

    //Set for all PIDS an error tolerance of 1.5 cm
    k1Controller.setTolerance(ElevatorConstants.ERROR_TOLERANCE);
    k2Controller.setTolerance(ElevatorConstants.ERROR_TOLERANCE);
    k3Controller.setTolerance(ElevatorConstants.ERROR_TOLERANCE);

    leaderConfig = new SparkMaxConfig();
    followerConfig = new SparkMaxConfig();

    leader = new SparkMax(ElevatorConstants.CAN_ID_LEADER, MotorType.kBrushless);
    follower = new SparkMax(ElevatorConstants.CAN_ID_SLAVE, MotorType.kBrushless);

    leaderEncoder = leader.getEncoder();
    followerEncoder = follower.getEncoder();

    resetEncoders();

    burnFlash();

  }

  public void resetEncoders(){
    leaderEncoder.setPosition(0);
    followerEncoder.setPosition(0);
  }

  private void burnFlash(){

    leader.setCANTimeout(250);

    leaderConfig.idleMode(IdleMode.kCoast).
    inverted(ElevatorConstants.leaderInverted);
    
    leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leader.setCANTimeout(0);

    follower.setCANTimeout(250);

    followerConfig.idleMode(IdleMode.kCoast).
    follow(ElevatorConstants.CAN_ID_LEADER, ElevatorConstants.slaveInverted);
 
    follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    follower.setCANTimeout(0);

  }

  @Override
  public void periodic(){

    SmartDashboard.putNumber("Leader pos", getLeaderPosition());
    SmartDashboard.putNumber("Slave pos", getFollowerPosition());
    SmartDashboard.putNumber("Average pos", getAveragePosition());

    SmartDashboard.putNumber("PIDOutput", leader.getAppliedOutput());
    SmartDashboard.putNumber("PIDNumber", getSetpoint());

    Domain retract2 = new Domain(ElevatorConstants.SETPOINT_RETRACT, ElevatorConstants.SETPOINT_L2);
    Domain retactl3 = new Domain(ElevatorConstants.SETPOINT_L2, ElevatorConstants.SETPOINT_L3);
    Domain retactl4 = new Domain(ElevatorConstants.SETPOINT_L3, ElevatorConstants.SETPOINT_L4);

    if (retract2.inRange(getLeaderPosition())) {
        tracker = ELEVATOR_LEVELS.k1;
    }

    if (retactl3.inRange(getLeaderPosition())) {
        tracker = ELEVATOR_LEVELS.k2;
    }

    if (retactl4.inRange(getLeaderPosition())) {
        tracker = ELEVATOR_LEVELS.k3;
    }

    SmartDashboard.putString("PIDLevels", tracker.toString());

  }

  public double getLeaderPosition(){
    return leaderEncoder.getPosition() * ElevatorConstants.CONVERSION_FACTOR + ElevatorConstants.ELEVATOR_OFFSET;
  }
  public double getFollowerPosition(){
    return followerEncoder.getPosition() * ElevatorConstants.CONVERSION_FACTOR + ElevatorConstants.ELEVATOR_OFFSET;
  }

  public double getAveragePosition(){
    double avg = (getLeaderPosition() + getFollowerPosition()) / 2;
    return avg;
  }

  public double getSetpoint(){
    return currentSetpoint;
  }

  public void runSpeed(double speed){
    leader.set(speed);
  }
  public void runRequest(double height){
    currentSetpoint = height;

    //Run for different PID control
    switch (tracker) {
        case k1:
        leader.set(k1Controller.calculate(getLeaderPosition(), height));
            break;
        case k2:
        leader.set(k2Controller.calculate(getLeaderPosition(), height));
            break;

        case k3:
        leader.set(k3Controller.calculate(getLeaderPosition(), height));
            break;
        default:
            break;
    }
  }
  public void retract(){
    runRequest(ElevatorConstants.SETPOINT_RETRACT);
  }
  public void toL2(){
    runRequest(ElevatorConstants.SETPOINT_L2);
  }
  public void toL3(){
    runRequest(ElevatorConstants.SETPOINT_L3);
  }
  public void toL4(){
    runRequest(ElevatorConstants.SETPOINT_L4);
  }
  public void toFeeder(){
    runRequest(ElevatorConstants.SETPOINT_FEEDER);
  }
  public boolean atGoal(){
    return Math.abs(getLeaderPosition() - currentSetpoint) <= ElevatorConstants.ERROR_TOLERANCE;
  }

  public void stop(){
    leader.stopMotor();
  }
  
}
