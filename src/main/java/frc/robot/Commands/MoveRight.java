package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Drive.swerve;

public class MoveRight extends Command {
    private final swerve drive;
    private final double distanceMeters;
    private final PIDController yController;
    private double targetY;

    public MoveRight(swerve drive, double distanceMeters) {
    
        this.drive = drive;
        this.distanceMeters = distanceMeters;
        this.yController = new PIDController(
            DriveConstants.yGains.getP(),
            DriveConstants.yGains.getI(),
            DriveConstants.yGains.getD()
        );
        yController.setTolerance(0.01); // 1cm tolerance
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // initial target position
        targetY = drive.getY() + distanceMeters;

        SmartDashboard.putNumber("MoveRight: Initial Y", drive.getY());
        SmartDashboard.putNumber("MoveRight: Target Y", targetY);
        SmartDashboard.putBoolean("MoveRight: At Setpoint", false);
    }

    @Override
    public void execute() {

        double ySpeed = yController.calculate(drive.getY(), targetY);
        ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);

        // Apply deadband
        if (Math.abs(ySpeed) < 0.05) {
            ySpeed = Math.copySign(0.05, ySpeed);
        }

        drive.runRobotRelative(0,ySpeed,0);

        SmartDashboard.putNumber("MoveRight: Current Y", drive.getY());
        SmartDashboard.putNumber("MoveRight: Target Y", targetY);
        SmartDashboard.putBoolean("MoveRight: At Setpoint", yController.atSetpoint());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drive.getY() - targetY) < 0.01;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        drive.stop();

        // Log final state
        SmartDashboard.putBoolean("MoveRight: Command Interrupted", interrupted);
        SmartDashboard.putNumber("MoveRight: Final Y", drive.getY());
    }
}
