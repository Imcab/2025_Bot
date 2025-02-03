package frc.robot.Subsystems.Drive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.DriveConstants;
import frc.robot.lib.SwerveConfig;


public class ModuleSpark {
    SparkMax driveSparkMax, turnSparkMax;
    RelativeEncoder enc_drive, enc_turn;

    AnalogInput AbsoluteEncoder;

    boolean isDriveMotorInverted;
    boolean isTurnMotorInverted;

    SparkMaxConfig config_drive, config_turn;


    private final PIDController turnPID;
    private final PIDController drivePID;
    private final SimpleMotorFeedforward drivFeedforward;
    private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
    private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop

    double Offset;

    public ModuleSpark(int index){

        config_drive = new SparkMaxConfig();
        config_turn = new SparkMaxConfig();

        drivePID = new PIDController(0.05, 0.0, 0.0);
        drivFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
        turnPID = new PIDController(6.2, 0.0, 0.0);

        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        switch (index) {
          case 0:
            driveSparkMax = new SparkMax(DriveConstants.frontLeft.DrivePort, MotorType.kBrushless);
            turnSparkMax = new SparkMax(DriveConstants.frontLeft.TurnPort, MotorType.kBrushless);
            AbsoluteEncoder = new AnalogInput(DriveConstants.frontLeft.EncPort);
            isDriveMotorInverted = DriveConstants.frontLeft.DrivemotorReversed;
            isTurnMotorInverted = DriveConstants.frontLeft.TurnmotorReversed;
            Offset = DriveConstants.frontLeft.offset;
  
            
            break;
          case 1:
            driveSparkMax = new SparkMax(DriveConstants.frontRight.DrivePort, MotorType.kBrushless);
            turnSparkMax = new SparkMax(DriveConstants.frontRight.TurnPort, MotorType.kBrushless);
            AbsoluteEncoder = new AnalogInput(DriveConstants.frontRight.EncPort);
            isDriveMotorInverted = DriveConstants.frontRight.DrivemotorReversed;
            isTurnMotorInverted = DriveConstants.frontRight.TurnmotorReversed;
            Offset = DriveConstants.frontRight.offset; 
            

            break;
          case 2:
            driveSparkMax = new SparkMax(DriveConstants.backLeft.DrivePort, MotorType.kBrushless);
            turnSparkMax = new SparkMax(DriveConstants.backLeft.TurnPort, MotorType.kBrushless);
            AbsoluteEncoder = new AnalogInput(DriveConstants.backLeft.EncPort);
            isDriveMotorInverted = DriveConstants.backLeft.DrivemotorReversed;
            isTurnMotorInverted = DriveConstants.backLeft.TurnmotorReversed;
            Offset = DriveConstants.backLeft.offset;
            

            break;
          case 3:
            driveSparkMax = new SparkMax(DriveConstants.backRight.DrivePort, MotorType.kBrushless);
            turnSparkMax = new SparkMax(DriveConstants.backRight.TurnPort, MotorType.kBrushless);
            AbsoluteEncoder = new AnalogInput(DriveConstants.backRight.EncPort);
            isDriveMotorInverted = DriveConstants.backRight.DrivemotorReversed;
            isTurnMotorInverted = DriveConstants.backRight.TurnmotorReversed;
            Offset = DriveConstants.backRight.offset;
            

            break;
          default:
            throw new RuntimeException("Invalid module index");
        }

        driveSparkMax.setCANTimeout(250);
        turnSparkMax.setCANTimeout(250);

        enc_drive = driveSparkMax.getEncoder();
        enc_turn = driveSparkMax.getEncoder();

        config_drive();
        config_turn();

       
    }

    public void config_drive(){
        config_drive
        .inverted(isDriveMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .voltageCompensation(12);
  
        config_drive.encoder
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(10);
  
        driveSparkMax.configure(config_drive, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }
  
      public void config_turn(){
        
        config_turn
        .inverted(isTurnMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20)
        .voltageCompensation(12);
  
        config_turn.encoder
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(10);

        turnSparkMax.configure(config_turn, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
        
      }

    public void periodic(){
        if (angleSetpoint != null) {
            turnSparkMax.setVoltage(
                turnPID.calculate(getRotation().getRadians(), angleSetpoint.getRadians()));
      
            // Run closed loop drive control
            // Only allowed if closed loop turn control is running
            if (speedSetpoint != null) {
              // Scale velocity based on turn error
              //
              // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
              // towards the setpoint, its velocity should increase. This is achieved by
              // taking the component of the velocity in the direction of the setpoint.
              double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnPID.getPositionError());
      
              // Run drive controller
              double velocityRadPerSec = adjustSpeedSetpoint / SwerveConfig.measures.WHEELRADIUS;
              driveSparkMax.setVoltage(
                  drivFeedforward.calculate(velocityRadPerSec)
                      + drivePID.calculate(Units.rotationsPerMinuteToRadiansPerSecond(enc_drive.getVelocity()) / SwerveConfig.reductions.DriveReduction, velocityRadPerSec));
            }
          }

    }

    public double getDegrees(){
      double encoderBits = AbsoluteEncoder.getValue();
      double angleEncoder = (encoderBits * 360) / 4096;

      return angleEncoder - Offset;
    }
  
    public Rotation2d getRotation(){
      return Rotation2d.fromDegrees(getDegrees());
    }

    public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null

    state.optimize(getRotation());
  
    angleSetpoint = state.angle;
    speedSetpoint = state.speedMetersPerSecond;

    return state;
  }

  public void setSpeed(SwerveModuleState desiredState){
    driveSparkMax.setVoltage(desiredState.speedMetersPerSecond);
  }

  public double getDrivePositionMeters(){
    return Units.rotationsToRadians(enc_drive.getPosition()) / SwerveConfig.reductions.DriveReduction * SwerveConfig.measures.WHEELRADIUS;
  }  
  public double getDriveVelocityMetersxSec(){
    return Units.rotationsPerMinuteToRadiansPerSecond(enc_drive.getVelocity()) / SwerveConfig.reductions.DriveReduction;
  }
  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDrivePositionMeters(), new Rotation2d(getRotation().getRadians()));
  }
  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocityMetersxSec(), new Rotation2d(getRotation().getRadians()));
  }

  public void stop() {
    driveSparkMax.set(0.0);
    turnSparkMax.set(0.0);
    // Disable closed loop control for turn and drive
    angleSetpoint = null;
    speedSetpoint = null;
  }





}