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
import frc.robot.lib.ClosedLoopControl;
import frc.robot.lib.SwerveConfig;
import frc.robot.lib.ClosedLoopControl.ClosedLoopRequest;
import frc.robot.lib.ClosedLoopControl.OutputType;
import frc.robot.lib.util.ModuleMap;


public class ModuleSpark {
    SparkMax driveSparkMax, turnSparkMax;
    RelativeEncoder enc_drive, enc_turn;

    AnalogInput AbsoluteEncoder;

    boolean isDriveMotorInverted;
    boolean isTurnMotorInverted;

    SparkMaxConfig config_drive, config_turn;


    //private final PIDController turnPID;
    private final ClosedLoopControl turnControl;
    private final ClosedLoopRequest turnRequest;
    private final PIDController drivePID;
    private final SimpleMotorFeedforward drivFeedforward;
    private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
    private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop
    private ModuleMap map;

    double Offset;

    public ModuleSpark(int index){

        config_drive = new SparkMaxConfig();
        config_turn = new SparkMaxConfig();

        drivePID = new PIDController(0.05, 0.0, 0.0);
        drivFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
        //turnPID = new PIDController(6.2, 0.0, 0.0);

        turnControl = new ClosedLoopControl(DriveConstants.turnGains, OutputType.kPositive);

        turnRequest =  turnControl.new ClosedLoopRequest();


        turnControl.enableContinuousInput(Math.PI);

        turnControl.initTuning("TurnTune");

        //turnPID.enableContinuousInput(-Math.PI, Math.PI);

        switch (index) {
          case 0:
            map = DriveConstants.backRight.br;
            driveSparkMax = new SparkMax(map.drive, MotorType.kBrushless);
            turnSparkMax = new SparkMax(map.turn, MotorType.kBrushless);
            AbsoluteEncoder = new AnalogInput(map.encoder);
            isDriveMotorInverted = map.driveInv;
            isTurnMotorInverted = map.turnInv;
            Offset = map.off;
            
            break;
          case 1:
            map = DriveConstants.backLeft.bl;
            driveSparkMax = new SparkMax(map.drive, MotorType.kBrushless);
            turnSparkMax = new SparkMax(map.turn, MotorType.kBrushless);
            AbsoluteEncoder = new AnalogInput(map.encoder);
            isDriveMotorInverted = map.driveInv;
            isTurnMotorInverted = map.turnInv;
            Offset = map.off;
            

            break;
          case 2:
            map = DriveConstants.frontRight.fr;
            driveSparkMax = new SparkMax(map.drive, MotorType.kBrushless);
            turnSparkMax = new SparkMax(map.turn, MotorType.kBrushless);
            AbsoluteEncoder = new AnalogInput(map.encoder);
            isDriveMotorInverted = map.driveInv;
            isTurnMotorInverted = map.turnInv;
            Offset = map.off;
            

            break;
          case 3:
            map = DriveConstants.frontLeft.fl;
            driveSparkMax = new SparkMax(map.drive, MotorType.kBrushless);
            turnSparkMax = new SparkMax(map.turn, MotorType.kBrushless);
            AbsoluteEncoder = new AnalogInput(map.encoder);
            isDriveMotorInverted = map.driveInv;
            isTurnMotorInverted = map.turnInv;
            Offset = map.off;
            

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
        .idleMode(IdleMode.kCoast)
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
      turnControl.tuneWithInterface();
      turnControl.graph("TurnGraph");

        if (angleSetpoint != null) {
            turnSparkMax.setVoltage(
                //turnPID.calculate(AngleEncoder().getRadians(), angleSetpoint.getRadians()));

                turnControl.runRequest(turnRequest.withReference(AngleEncoder().getRadians()).toSetpoint(angleSetpoint.getRadians())));
      
            // Run closed loop drive control
            // Only allowed if closed loop turn control is running
            if (speedSetpoint != null) {
              // Scale velocity based on turn error
              //
              // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
              // towards the setpoint, its velocity should increase. This is achieved by
              // taking the component of the velocity in the direction of the setpoint.
              double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnControl.getCurrentError());
      
              // Run drive controller
              double velocityRadPerSec = adjustSpeedSetpoint / SwerveConfig.measures.WHEELRADIUS;
              driveSparkMax.setVoltage(
                  drivFeedforward.calculate(velocityRadPerSec)
                      + drivePID.calculate(Units.rotationsPerMinuteToRadiansPerSecond(enc_drive.getVelocity()) / SwerveConfig.reductions.DriveReduction, velocityRadPerSec));
            }
          }

    }
  
    public Rotation2d AngleEncoder(){

      double encoderBits = AbsoluteEncoder.getValue();
      double angleEncoder = (encoderBits * 360) / 4096;

      return Rotation2d.fromDegrees(angleEncoder - Offset);

    }

    public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null

    state.optimize(AngleEncoder());
  
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
    return new SwerveModulePosition(getDrivePositionMeters(), AngleEncoder());
  }
  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocityMetersxSec(), AngleEncoder());
  }

  public void stop() {
    driveSparkMax.set(0.0);
    turnSparkMax.set(0.0);
    // Disable closed loop control for turn and drive
    angleSetpoint = null;
    speedSetpoint = null;
  }

}