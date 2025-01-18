// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveCommands;
import frc.robot.Commands.MoveRight;
import frc.robot.Subsystems.Superstructure;

public class RobotContainer {

  CommandXboxController controller = new CommandXboxController(0);
  CommandXboxController controller2 = new CommandXboxController(1);
  frc.robot.Subsystems.Drive.swerve swerve;
  Superstructure superstructure;
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  

  public RobotContainer() {
    swerve = new frc.robot.Subsystems.Drive.swerve();
    superstructure = new Superstructure();

    configureBindings();
  }

  private void configureBindings() {

    //Drive modes. Normal Drive
    swerve.setDefaultCommand(DriveCommands.joystickDrive(swerve, ()-> controller.getLeftY(),  ()-> controller.getLeftX(),  ()-> -controller.getRightX()));
    //Drive modes. Slowed Drive
    controller.leftBumper().whileTrue(DriveCommands.joystickDrive(swerve, ()-> controller.getLeftY() * 0.25,  ()-> controller.getLeftX() * 0.25,  ()-> -controller.getRightX() * 0.25));

   controller.start().whileTrue(DriveCommands.resetHeading(swerve)); //resets heading
   controller.leftStick().whileTrue(DriveCommands.brake(swerve)); //Stops the swerve in an "X" pattern position

    //start alignment with limelight
   controller.b().whileTrue(DriveCommands.getInRange(swerve, ()-> controller.getLeftX()));

   controller.rightStick().whileTrue(new MoveRight(swerve, 0.3302));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}