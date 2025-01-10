package frc.robot.lib;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class SwerveConfig{

    public class speeds {
        public static final double MAX_SPEED_MPS = 5.7912;
        public static final double MAX_LINEAR_SPEED = Units.feetToMeters(19.0);
        public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / measures.DRIVE_BASE_RADIUS;
        public static final double LIMMaxSpeed = 3.0; // 3 meters per second
        public static final double LIMMaxAngularSpeed = Math.PI;
    }
    
    public class measures{
        public static final double WHEELRADIUS = Units.inchesToMeters(2.0);
        public static final double WHEELDIAMETER = Units.inchesToMeters(4.0);
        public static final double TRACK_WIDTH_X = Units.inchesToMeters(28); 
        public static final double TRACK_WIDTH_Y = Units.inchesToMeters(31.7);

        public static final double robotMassKg = 68.178;
        public static final double robotMOI = 3.177;
        public static final double wheelCOF = 1.0;

        public static final double DRIVE_BASE_RADIUS =
            Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
        
        public static Translation2d[] getTranslations() {
            return new Translation2d[] {
                new Translation2d(TRACK_WIDTH_X/2.0, TRACK_WIDTH_Y/2.0),
                new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
                new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
            };
        }
        
    }

    public class reductions{
        public static final double DriveReduction = 5.36;
        public static final double TurnReduction = 18.75;  
    }

    public static final RobotConfig ppConfig =
      new RobotConfig(
          measures.robotMassKg,
          measures.robotMOI,
          new ModuleConfig(
              measures.WHEELRADIUS,
              speeds.MAX_SPEED_MPS,
              measures.wheelCOF,
              DCMotor.getNEO(1).withReduction(reductions.DriveReduction),
              30,
              1),
          measures.getTranslations());

}
