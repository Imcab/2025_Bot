package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.lib.Alerts;
import frc.robot.lib.SwerveConfig;
import frc.robot.lib.util.Actions;
import frc.robot.lib.vision.LimelightHelpers;
import frc.robot.lib.vision.vision.limelight;

public class DriveTrain extends SubsystemBase{

    private final AHRS navX =  new AHRS(NavXComType.kMXP_SPI);

    //private final limelight lim;
    
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConfig.measures.getTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
    private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    ModuleSpark []modules  = new ModuleSpark[4];

    public DriveTrain(){

        AutoBuilder.configure(this::getPose, this::setPose, this::getChassisSpeeds , this::runVelocity,  new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)), SwerveConfig.ppConfig, () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, this);
        

        modules[0] = new ModuleSpark(0);
        modules[1] = new ModuleSpark(1);
        modules[2] = new ModuleSpark(2);
        modules[3] = new ModuleSpark(3);

        new Thread(() -> {
            try{
                Thread.sleep(1000);
                resetHeading();
            } catch (Exception e){
    
            }
          }).start();
    }

    public void periodic(){

      //State and notification actions:

      SmartDashboard.putNumber("Period", RobotState.getPeriod());
      
      SmartDashboard.putBoolean("target", LimelightHelpers.getTV("limelight-buluk"));

      RobotState.setAngularVelocity(Units.radiansToDegrees(getYawVelicityRadPerSec()));

      Actions.runOnce(()-> RobotState.isAngularVelAboveLimits(), ()-> Alerts.sendFastSpeed());
      
      for (var module : modules) {
        module.periodic();
      }
      if (DriverStation.isDisabled()) {
        for (var module : modules) {
          module.stop();
      }}

      SwerveModulePosition[] modulePositions = getModulePositions();
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (navX.isConnected() == true) {
        // Use the real gyro angle
        rawGyroRotation = getnavXRotation();
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply odometry update
      poseEstimator.update(rawGyroRotation, modulePositions);
    }

    public double getAngle(){
      return Math.IEEEremainder(navX.getAngle(), 360);
    }

    public Rotation2d getnavXRotation(){
      return Rotation2d.fromDegrees(getAngle());
    }

  
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConfig.speeds.MAX_LINEAR_SPEED);

    for (int i = 0; i < 4; i++) {
        modules[i].runSetpoint(setpointStates[i]);
    }
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = SwerveConfig.measures.getTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Double getX() {
    return getPose().getX();
  }

  public Double getY() {
    return getPose().getY();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  public ChassisSpeeds getChassisSpeeds(){
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return SwerveConfig.speeds.MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return SwerveConfig.speeds.MAX_ANGULAR_SPEED;
  }

  public void resetHeading(){

    navX.reset();
    
  }

  public double getYawVelicityRadPerSec(){
    return Units.degreesToRadians(-navX.getRawGyroZ());
  }

  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  public double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .030;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight-buluk") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= SwerveConfig.speeds.LIMMaxAngularSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= 1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  public double limelight_range_proportional()
  {    
    double kP = .04;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight-buluk") * kP;
    targetingForwardSpeed *= SwerveConfig.speeds.LIMMaxSpeed;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;

  }

  public void driveVision(double x, double y, double rot, double period){
    var State = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(new ChassisSpeeds(x,y,rot), period));

    SwerveDriveKinematics.desaturateWheelSpeeds(State, SwerveConfig.speeds.LIMMaxSpeed);

    modules[0].setSpeed(State[0]);
    modules[1].setSpeed(State[1]);
    modules[2].setSpeed(State[2]);
    modules[3].setSpeed(State[3]);
  }


  
}
