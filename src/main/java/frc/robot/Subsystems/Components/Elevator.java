// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Components;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  //Crea los dos motores y la comfiguracion para los dos
  public TalonFX Leader;
  public TalonFX Slave;
  public TalonFXConfiguration configLeader, configSlave;
  
  //Constructor o "Init"
  public Elevator() {

    //Asigna valores
    Leader = new TalonFX(ElevatorConstants.CAN_ID_LEADER);
    Slave = new TalonFX(ElevatorConstants.CAN_ID_SLAVE);

    configLeader = new TalonFXConfiguration();
    configSlave = new TalonFXConfiguration();

    //Carga y actualiza las configuraciones al iniciarse el subsistema
    flashConfigs();

    /*Esto hace que el motor esclavo siga siempre al motor Leader sin tener que mandarle una señal:
    Ej: si yo pongo en el código Leader.set(0.5), el motor "Slave tambien se moverá a 0.5 sin necesidad de hacer nada"
    */
    Slave.setControl(new StrictFollower(Leader.getDeviceID())); 
  }

  //Función para llamar en el constructor donde se aplicará toda la configuración de los motores
  private void flashConfigs(){

    //restore factory defaults
    Leader.getConfigurator().apply(new TalonFXConfiguration());
    Slave.getConfigurator().apply(new TalonFXConfiguration());

    //Aqui se decide si se invierten o no los motores al igual q los pone para q frenen de golpe
    configLeader.MotorOutput.Inverted = ElevatorConstants.leaderIV;
    configSlave.MotorOutput.Inverted = ElevatorConstants.slaveIV;
    configLeader.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configSlave.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    var pidGains = configLeader.Slot0;

    //Constantes del PID
    pidGains.kP = ElevatorConstants.motionMagicGains.getP();
    pidGains.kI = ElevatorConstants.motionMagicGains.getI();
    pidGains.kD = ElevatorConstants.motionMagicGains.getD();
    pidGains.kS = ElevatorConstants.motionMagicGains.getS();
    pidGains.kV = ElevatorConstants.motionMagicGains.getV();
    pidGains.kA = ElevatorConstants.motionMagicGains.getAcceleration();
    
    var motionMagicGains = configLeader.MotionMagic;

    //valores más específicos del motion magic
    motionMagicGains.MotionMagicAcceleration = ElevatorConstants.MM_Acc;
    motionMagicGains.MotionMagicCruiseVelocity = ElevatorConstants.MM_CruiseVel;
    motionMagicGains.MotionMagicJerk = ElevatorConstants.MM_Jerk;
    motionMagicGains.MotionMagicExpo_kA = ElevatorConstants.MM_ExpoKa;
    motionMagicGains.MotionMagicExpo_kV = ElevatorConstants.MM_ExpoKv;

    //Carga y aplica las configuraciones
    Leader.getConfigurator().apply(configLeader);
    Slave.getConfigurator().apply(configSlave);
  }

  //Método periódico
  @Override
  public void periodic() {

  }
}
