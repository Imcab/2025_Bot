package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.lib.SwerveConfig;
import frc.robot.lib.util.SwerveEncoder;
import frc.robot.lib.util.SwerveEncoder.module;


public class ModuleSpark{
    SparkMax driveSparkMax, turnSparkMax;
    RelativeEncoder enc_drive, enc_turn;
    
    SwerveEncoder module_Encoder;

    boolean isDriveMotorInverted;
    boolean isTurnMotorInverted;

    private final PIDController turnPID;
    private final PIDController drivePID;
    private final SimpleMotorFeedforward drivFeedforward;
    private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
    private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop

    SparkMaxConfig config_drive, config_turn;

    public ModuleSpark(int index){
        config_drive = new SparkMaxConfig();
        config_turn = new SparkMaxConfig();

        drivePID = new PIDController(DriveConstants.driveGains.getP(), DriveConstants.driveGains.getI(), DriveConstants.driveGains.getD());
        drivFeedforward = new SimpleMotorFeedforward(DriveConstants.driveGains.getS(), DriveConstants.driveGains.getV());
        turnPID = new PIDController(DriveConstants.turnGains.getP(), DriveConstants.turnGains.getI(), DriveConstants.turnGains.getD());

        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        switch (index) {
          case 0:
            driveSparkMax = new SparkMax(DriveConstants.frontLeft.DrivePort, MotorType.kBrushless);
            turnSparkMax = new SparkMax(DriveConstants.frontLeft.TurnPort, MotorType.kBrushless);
            module_Encoder = new SwerveEncoder(module.FL,DriveConstants.frontLeft.EncPort);
            isDriveMotorInverted = DriveConstants.frontLeft.DrivemotorReversed;
            isTurnMotorInverted = DriveConstants.frontLeft.TurnmotorReversed;

            module_Encoder.setOffset(DriveConstants.frontLeft.offset);
            
            
            break;
          case 1:
            driveSparkMax = new SparkMax(DriveConstants.frontRight.DrivePort, MotorType.kBrushless);
            turnSparkMax = new SparkMax(DriveConstants.frontRight.TurnPort, MotorType.kBrushless);
            module_Encoder = new SwerveEncoder(module.FR,DriveConstants.frontRight.EncPort);
            isDriveMotorInverted = DriveConstants.frontRight.DrivemotorReversed;
            isTurnMotorInverted = DriveConstants.frontRight.TurnmotorReversed;

            module_Encoder.setOffset(DriveConstants.frontRight.offset); 
        
            break;
          case 2:
            driveSparkMax = new SparkMax(DriveConstants.backLeft.DrivePort, MotorType.kBrushless);
            turnSparkMax = new SparkMax(DriveConstants.backLeft.TurnPort, MotorType.kBrushless);
            module_Encoder = new SwerveEncoder(module.BL,DriveConstants.backLeft.EncPort);
            isDriveMotorInverted = DriveConstants.backLeft.DrivemotorReversed;
            isTurnMotorInverted = DriveConstants.backLeft.TurnmotorReversed;

            module_Encoder.setOffset(DriveConstants.backLeft.offset);
            

            break;
          case 3:
            driveSparkMax = new SparkMax(DriveConstants.backRight.DrivePort, MotorType.kBrushless);
            turnSparkMax = new SparkMax(DriveConstants.backRight.TurnPort, MotorType.kBrushless);
            module_Encoder = new SwerveEncoder(module.BR,DriveConstants.backRight.EncPort);
            isDriveMotorInverted = DriveConstants.backRight.DrivemotorReversed;
            isTurnMotorInverted = DriveConstants.backRight.TurnmotorReversed;

            module_Encoder.setOffset(DriveConstants.backRight.offset);

            break;
          default:
            throw new RuntimeException("Invalid module index");
        }

        driveSparkMax.setCANTimeout(250);
        turnSparkMax.setCANTimeout(250);

        config_drive();
        config_turn();

        enc_drive = driveSparkMax.getEncoder();
        enc_turn = driveSparkMax.getEncoder();

        enc_drive.setPosition(0.0);
        enc_turn.setPosition(0.0);

        driveSparkMax.setCANTimeout(0);
        turnSparkMax.setCANTimeout(0);

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
  
        turnSparkMax.configure(config_turn, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }
  
      public void config_turn(){
        
        config_turn
        .inverted(isTurnMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .voltageCompensation(12);
  
        config_drive.encoder
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(10);
  
        driveSparkMax.configure(config_drive, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }

    public boolean isConnected(){
      return module_Encoder.isConnected();
    }
    public void autoAdjust(){
      module_Encoder.adjustOffset();
    }

    public String getModuleName(){
      return module_Encoder.getName();
    }

    public int getChannel(){
      return module_Encoder.getPort();
    }

    public double getVoltage(){
      return module_Encoder.getVoltage();
    }

    public void periodic(){
        if (angleSetpoint != null) {
            turnSparkMax.setVoltage(-
                turnPID.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));
      
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

    public Rotation2d getAngle(){

      if (!isConnected()) {
        return new Rotation2d(enc_turn.getPosition() / SwerveConfig.reductions.TurnReduction);
      }

      return module_Encoder.getRotation2D();
    }

    public double getInternalEncoderPosition(){
      return enc_turn.getPosition() / SwerveConfig.reductions.TurnReduction;
    }

    public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    state.optimize(getAngle());

    // Update setpoints, controllers run in "periodic"
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
    return new SwerveModulePosition(getDrivePositionMeters(), getAngle());
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocityMetersxSec(), getAngle());
  }

  public void stop() {
    driveSparkMax.stopMotor();
    turnSparkMax.stopMotor();
    // Disable closed loop control for turn and drive
    angleSetpoint = null;
    speedSetpoint = null;
  }

}