// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Commands.k;
import frc.robot.Commands.m;
import frc.robot.Commands.DriveCommands.DriveCommands;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Components.Elevator;
//import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Drive.swerve;

public class RobotContainer {

  //For autonomous
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController c = new CommandXboxController(1);
  private final swerve swerve;
  private final Elevator e;
  //private final Superstructure superstructure;
  
  public RobotContainer() {
    swerve = new swerve();
    e = new Elevator();
    //superstructure = new Superstructure();
    
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
  

    c.leftBumper().whileTrue(new m(e, ()-> c.getLeftY() * -0.2));
    c.b().whileTrue(new k(e, ElevatorConstants.SETPOINT_FEEDER));
    c.x().whileTrue(new k(e, ElevatorConstants.SETPOINT_L2));
    c.y().whileTrue(new k(e, ElevatorConstants.SETPOINT_L3));
    c.rightBumper().whileTrue(new k(e, ElevatorConstants.SETPOINT_L4));
    c.a().whileTrue(new k(e, 66));
    c.start().whileTrue(new InstantCommand(()->{e.resetEncoders();}, e));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("a");
  }

}