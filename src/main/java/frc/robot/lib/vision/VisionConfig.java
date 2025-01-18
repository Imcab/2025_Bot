package frc.robot.lib.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.lib.SwerveConfig;
import frc.robot.lib.util.QoLUtil;

public class VisionConfig {
    
    public static class limelight {
        public static final String name = "limelight-buluk";
        public static final int forwardCoefficient = 1; // if the robot never turns in the correct direction, kP should be inverted.
        public static final int aimCoefficient = -1;
        public static final boolean useMegatag2 = true;
        public static final double forwardKp = 0.2;
        public static final double angularKp = 0.01158;
        public static final boolean useTAforRange = true;
        public static final Matrix<N3,N1> trust = VecBuilder.fill(0.7, 0.7, 99999);

        public static final double TrackMaxSpeed = QoLUtil.percentageOf(90, SwerveConfig.speeds.MAX_LINEAR_SPEED);
        public static final double TrackMaxAngularSpeed = Math.PI;
        
    }
    public static class photonvision {
        public static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        public static final String backLeft = "backLeft";
        public static final String backRight = "backRight";
        public static final Transform3d robotToBackLeft = new Transform3d(0, 0, 0, null);
        public static final Transform3d robotToBackRight = new Transform3d(0, 0, 0, null);
        public static final boolean driveModeBL = false;
        public static final boolean driveModeBR = false;
    }

}
