// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Components;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsHanger;
import frc.robot.Constants.ElevatorConstants;

public class Hanger extends SubsystemBase {

    private final SparkMax RightHanger, LeftHanger;
    private double leftPosition;
    private double rightPosition;
    private final DigitalInput limitSwitch;
    private SparkMaxConfig config1, config2;

    public Hanger() {
        
        RightHanger = new SparkMax(ConstantsHanger.RightHangerPort, MotorType.kBrushless);
        LeftHanger = new SparkMax(ConstantsHanger.LeftHangerPort, MotorType.kBrushless);

        limitSwitch = new DigitalInput(ConstantsHanger.switchPort);

        burnFlash();
    }

    private void burnFlash() {

    RightHanger.setCANTimeout(250);
    config1.idleMode(IdleMode.kCoast).inverted(ConstantsHanger.r_inverted);
    RightHanger.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    RightHanger.setCANTimeout(0);

    LeftHanger.setCANTimeout(250);
    config1.idleMode(IdleMode.kCoast).inverted(ConstantsHanger.l_inverted);
    LeftHanger.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    LeftHanger.setCANTimeout(0);
    
    }

    @Override
    public void periodic(){
   
    }

    public boolean limitPressed(){
        return !limitSwitch.get();
    }
 
    public void setSpeed(double speed){
      RightHanger.set(speed);
      LeftHanger.set(speed);

    }

    public void stopAll(){
        RightHanger.stopMotor();
        LeftHanger.stopMotor();
    }
}
