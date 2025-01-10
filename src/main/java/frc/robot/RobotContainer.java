// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveCommands;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Superstructure;
import frc.robot.lib.Alerts;

public class RobotContainer {

  CommandXboxController controller = new CommandXboxController(0);
  CommandXboxController controller2 = new CommandXboxController(1);
  DriveTrain swerve;
  Superstructure superstructure;
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  

  public RobotContainer() {
    swerve = new DriveTrain();
    superstructure = new Superstructure();

    configureBindings();
  }

  private void configureBindings() {

    //------------------------------------------- DRIVER 1 -------------------------------------------
    controller.start().whileTrue(DriveCommands.resetHeading(swerve).finallyDo(()-> Alerts.sendNavxReset())); //resets heading
    controller.back().whileTrue(DriveCommands.formX(swerve));

    swerve.setDefaultCommand(DriveCommands.joystickDrive(swerve, ()-> controller.getLeftY(),  ()-> controller.getLeftX(),  ()-> -controller.getRightX()));

    //fija el swerve a 0 grados
    controller.b().whileTrue(DriveCommands.getInRange(swerve, ()-> controller.getLeftX()));

    //------------------------------------------- DRIVER 1 -------------------------------------------


    //------------------------------------------- DRIVER 2 ------------------------------------------- 

    //------------------------------------------- DRIVER 2 ------------------------------------------- 


  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}