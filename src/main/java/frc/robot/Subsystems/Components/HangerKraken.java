// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Components;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsHanger;

public class HangerKraken extends SubsystemBase {

    private final TalonFX RightHanger, LeftHanger;
    private final TalonFXConfiguration LHangerConfiguration, RHangerConfiguration;
    private double leftPosition;
    private double rightPosition;

    public HangerKraken () {
        
        RightHanger = new TalonFX(ConstantsHanger.RightHangerPort);
        LeftHanger = new TalonFX(ConstantsHanger.LeftHangerPort);

        LHangerConfiguration = new TalonFXConfiguration();
        RHangerConfiguration = new TalonFXConfiguration();

        LHangerConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        RHangerConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        LHangerConfiguration.Slot0.kP = ConstantsHanger.hangerGains.getP();
        LHangerConfiguration.Slot0.kI = ConstantsHanger.hangerGains.getI();
        LHangerConfiguration.Slot0.kD = ConstantsHanger.hangerGains.getD();
    
        RHangerConfiguration.Slot0.kP = ConstantsHanger.hangerGains.getP();
        RHangerConfiguration.Slot0.kI = ConstantsHanger.hangerGains.getI();
        RHangerConfiguration.Slot0.kD = ConstantsHanger.hangerGains.getD();

        RightHanger.getConfigurator().apply(RHangerConfiguration);
        LeftHanger.getConfigurator().apply(LHangerConfiguration);

    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("LeftHangerPosition", leftPosition);
        SmartDashboard.putNumber("RightHangerPosition", leftPosition);

    }
 
    public void moverPID(double setpoint){
        //Ver en que esta el setpoint
        RightHanger.setControl(new PositionVoltage(setpoint));
        LeftHanger.setControl(new PositionVoltage(setpoint));
        // Establece la salida del PID en los motores
    }

    public double LeftHangerAngle () {
        //Falto asignar un valor a left y right position
        leftPosition = LeftHanger.getPosition().getValueAsDouble();
        return leftPosition * 360;
    
    }

    public double RightHangerAngle() {
        rightPosition = RightHanger.getPosition().getValueAsDouble();
        return rightPosition * 360;
    }

    public void movemanual(double speed){
      RightHanger.setControl(new DutyCycleOut(speed));
      LeftHanger.setControl(new DutyCycleOut(speed));

      //si no funciona usar:
      //RightHanger.set(speed);
      //LeftHanger.set(speed);

    }

    public void stopAll(){
        RightHanger.stopMotor();
        LeftHanger.stopMotor();
    }
}
