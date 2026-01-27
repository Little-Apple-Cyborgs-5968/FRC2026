package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class Swerve {
        public static final double kRobotCentricSpeed = 0.5; // how fast robot centric drive speed is in m/s
    }

    public static class PathFinding {
        // Default path constraints for when pathfinding
        public static final PathConstraints kDefualtConstraints = new PathConstraints(
            4.0, 3.0,  // max velocity, max acceleration (m/s, m/s²)
            Units.degreesToRadians(540), Units.degreesToRadians(720)  // max angular vel, max angular accel
        );
    }

    public static class Vision {
        // ==================== LIMELIGHT NAMES ====================
        // Network table names for each Limelight (must match what's configured in the Limelight web interface)
        public static final String kLimelightFrontName = "limelight-two"; //left and two
        public static final String kLimelightBackName = "limelight-right";
        public static final String kLimelightGamepieceName = "limelight-gp";

        // ==================== CAMERA POSITIONS ====================
        // Camera positions relative to robot center (meters)
        // +X is forward, +Y is right, +Z is up
        // Rotation: Roll (X), Pitch (Y), Yaw (Z)
        
        // Front Limelight 4 - AprilTag camera (adjust these values to your robot)
        public static final Transform3d kLimelightFrontPosition = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(10.118),  // X: 10 inches forward from center
                Units.inchesToMeters(-9.770),   // Y: centered left-right
                Units.inchesToMeters(8.401)    // Z: 8 inches up from ground
            ),
            new Rotation3d(
                Units.degreesToRadians(180),                          // Roll: 0 degrees
                Units.degreesToRadians(65.0), // Pitch: degrees
                Units.degreesToRadians(25)      // Yaw: degrees
            )
        );

        // Back Limelight 4 - AprilTag camera (adjust these values to your robot)
        public static final Transform3d kLimelightBackPosition = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(10.118), // 
                Units.inchesToMeters(9.770),   // Y: left-right
                Units.inchesToMeters(8.401)    // Z: inches up from ground
            ),
            new Rotation3d(
                Units.degreesToRadians(0),                           // Roll: 0 degrees
                Units.degreesToRadians(65.0),  // Pitch: degrees
                Units.degreesToRadians(-25.0)  // Yaw: degrees
            )
        );

        // Gamepiece Limelight 2 - Neural network camera (adjust these values to your robot)
        public static final Transform3d kLimelightGamepiecePosition = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(12.0),  // X: 
                Units.inchesToMeters(0.0),   // Y: centered left-right
                Units.inchesToMeters(20.0)   // Z: 20 inches up from ground
            ),
            new Rotation3d(
                0.0,                          // Roll: 0 degrees
                Units.degreesToRadians(25.0), // Pitch: 25 degrees down to see floor
                0.0                           // Yaw: facing forward
            )
        );

        // ==================== FIELD MAP CONFIG ====================
        // Name of the .fmap file in the deploy directory
        // Download from: https://github.com/LimelightVision/limelern/releases (official 2026 field map)
        // Or export from PathPlanner field settings
        public static final String kFieldMapFileName = "FRC2026_WELDED.fmap";

        // ==================== POSE ESTIMATION CONFIG ====================
        // Standard deviations for vision measurements [x, y, theta]
        // Lower values = trust vision more, Higher values = trust odometry more
        
        // Single tag standard deviations (less accurate)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));
        
        // Multi-tag standard deviations (more accurate with MegaTag2)
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));

        // ==================== FILTERING THRESHOLDS ====================
        // Maximum distance to accept AprilTag readings (meters)
        public static final double kMaxTagDistance = 4.5;
        
        // Maximum ambiguity to accept AprilTag readings (0-1, lower is better)
        public static final double kMaxAmbiguity = 0.3;
        
        // Minimum tag area to accept readings (percentage of image)
        public static final double kMinTagArea = 0.1;

        // ==================== MEGATAG2 CONFIG ====================
        // Whether to use MegaTag2 (recommended for better accuracy)
        public static final boolean kUseMegaTag2 = true;

        // ==================== CAMERA POSE CONFIG ====================
        // Set to true to override web GUI camera poses with code values
        // Set to false to use values configured in Limelight web interface
        public static final boolean kSetCameraPosesFromCode = false;
    }
}
