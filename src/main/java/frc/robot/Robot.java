// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.Alerts;
import frc.robot.lib.util.Actions;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  
  public Robot() {
    m_robotContainer = new RobotContainer();

    //Actions.runOnce(()-> RobotState(), ()-> Alerts.sendNavxConnected());
    //Actions.runOnce(()-> !navX.isConnected(), ()-> Alerts.sendNavxDisconnected());

    Actions.runOnce(()-> RobotState.isLowBattery(), ()-> Alerts.sendLowBattery()); //Checa si la bateria no ha sido cambiada

    Actions.runOnce(()-> RobotState.isMediumCharge(), ()-> Alerts.sendMediumBattery()); //Empieza el juego con poca pila
    //Activa ahorro de energia en caso de iniciar con poca bateria 
    if (RobotState.isMediumCharge()) {
      RobotState.startwithLowMode(true);
    }
    
    RobotState.startwithLowMode(false);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    RobotState.setPeriod(getPeriod());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
