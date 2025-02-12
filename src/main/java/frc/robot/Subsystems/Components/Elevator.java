package frc.robot.Subsystems.Components;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.lib.tidal.Domain;
import frc.robot.lib.util.Gains;
import frc.robot.lib.ClosedLoopControl;
import frc.robot.lib.ClosedLoopControl.ClosedLoopRequest;

public class Elevator extends SubsystemBase {

  private final SparkMax leader, follower;
  private final SparkMaxConfig leaderConfig, followerConfig;
  private final RelativeEncoder leaderEncoder, followerEncoder;

  public enum ELEVATOR_LEVELS {
    k1, k2, k3
  }

  private ELEVATOR_LEVELS tracker = ELEVATOR_LEVELS.k1;

  // Use a single ClosedLoopControl instance
  private final ClosedLoopControl elevatorPID;

  private ClosedLoopRequest heightRequest;

  private Domain cero_To_L2 = new Domain(ElevatorConstants.SETPOINT_RETRACT, ElevatorConstants.SETPOINT_L2);
  private Domain L2_to_L3 = new Domain(ElevatorConstants.SETPOINT_L2, ElevatorConstants.SETPOINT_L3);
  private Domain l3_to_L4 = new Domain(ElevatorConstants.SETPOINT_L3, ElevatorConstants.SETPOINT_L4);

  public Elevator() {
    // Initialize ClosedLoopControl with default gains
    elevatorPID = new ClosedLoopControl(ElevatorConstants.k1_GAINS, ClosedLoopControl.OutputType.kPositive);

    heightRequest = elevatorPID.new ClosedLoopRequest();

    //Sets a tolerance and starts the tuner
    elevatorPID.setTolerance(ElevatorConstants.ERROR_TOLERANCE); 
    elevatorPID.initTuning("ElevatorTuner");

    leaderConfig = new SparkMaxConfig();
    followerConfig = new SparkMaxConfig();

    leader = new SparkMax(ElevatorConstants.CAN_ID_LEADER, MotorType.kBrushless);
    follower = new SparkMax(ElevatorConstants.CAN_ID_SLAVE, MotorType.kBrushless);

    leaderEncoder = leader.getEncoder();
    followerEncoder = follower.getEncoder();

    resetEncoders();
    burnFlash();
  }

  public void resetEncoders() {
    leaderEncoder.setPosition(0);
    followerEncoder.setPosition(0);
  }

  private void burnFlash() {
    leader.setCANTimeout(250);
    leaderConfig.idleMode(IdleMode.kCoast).inverted(ElevatorConstants.leaderInverted);
    leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leader.setCANTimeout(0);

    follower.setCANTimeout(250);
    followerConfig.idleMode(IdleMode.kCoast).follow(ElevatorConstants.CAN_ID_LEADER, ElevatorConstants.slaveInverted);
    follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    follower.setCANTimeout(0);
  }

  @Override
  public void periodic() {

    //Starts the graph and the tuner
    elevatorPID.graph("ElevatorPID");

    elevatorPID.tuneWithInterface();

    //Define ranges for different PID gains   
    if (cero_To_L2.inRange(getCentimeters())) {
        updateGains(ElevatorConstants.k1_GAINS);
        tracker = ELEVATOR_LEVELS.k1;
    }

    if (L2_to_L3.inRange(getCentimeters())) {
        updateGains(ElevatorConstants.k2_GAINS);
        tracker = ELEVATOR_LEVELS.k2;
    }

    if (l3_to_L4.inRange(getCentimeters())) {
        updateGains(ElevatorConstants.k3_GAINS);
        tracker = ELEVATOR_LEVELS.k3;
    }

    //Print the current Level
    SmartDashboard.putString("ElevatorLevel", tracker.toString());

    SmartDashboard.putNumber("CENT", getCentimeters());
  }

  private void updateGains(Gains newGains) {
    if (!elevatorPID.currentGains().equals(newGains)) { 
        elevatorPID.setGains(newGains);
    }
  }

  public double getCentimeters() {
    return leaderEncoder.getPosition() * (ElevatorConstants.CONVERSION_FACTOR / 3) + ElevatorConstants.ELEVATOR_OFFSET;
  }

  public double getSetpoint() {
    return elevatorPID.getSetpoint();
  }

  public void runSpeed(double speed) {
    leader.set(speed);
  }

  public void runRequest(double height) {
    //Runs a PID height Request
    leader.set(elevatorPID.runRequest(heightRequest.withReference(getCentimeters()).toSetpoint(height)));
  }

  public void retract() {
    runRequest(ElevatorConstants.SETPOINT_RETRACT);
  }

  public void toL2() {
    runRequest(ElevatorConstants.SETPOINT_L2);
  }

  public void toL3() {
    runRequest(ElevatorConstants.SETPOINT_L3);
  }

  public void toL4() {
    runRequest(ElevatorConstants.SETPOINT_L4);
  }

  public void toFeeder() {
    runRequest(ElevatorConstants.SETPOINT_FEEDER);
  }

  public boolean atGoal() {
    return elevatorPID.atGoal();
  }

  public void stop() {
    leader.stopMotor();
  }
}
