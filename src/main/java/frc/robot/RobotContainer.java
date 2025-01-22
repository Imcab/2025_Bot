// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveCommands.DriveCommands;
import frc.robot.Commands.DriveCommands.MoveRight;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Drive.swerve;

public class RobotContainer {

  //For autonomous
  public SendableChooser<Command> autoChooser = new SendableChooser<>();

  public final CommandXboxController controller = new CommandXboxController(0);
  public final CommandXboxController controller2 = new CommandXboxController(1);
  public final swerve swerve;
  public final Superstructure superstructure;
  
  public RobotContainer() {
    swerve = new swerve();
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
    //Align with right CORAL
    controller.rightStick().whileTrue(new MoveRight(swerve, 0.3302));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}