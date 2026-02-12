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
        
        // SYOMDrive (Synchronized Yaw-Optimized Motion Drive) constants
        public static final double kSYOMDriveMinVelocity = 0.33; // Minimum velocity (m/s) before rotation activates
        public static final double kSYOMDriveRotationKp = 8.0; // Proportional gain for smooth rotation to travel direction
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
        public static final String kLimelightLeftName = "limelight-two"; //left and two
        public static final String kLimelightRightName = "limelight-right"; // everytihn called back actually right
        public static final String kLimelightGamepieceName = "limelight-gp";

        // ==================== CAMERA POSITIONS ====================
        // Camera positions relative to robot center (meters)
        // +X is forward, +Y is right, +Z is up
        // Rotation: Roll (X), Pitch (Y), Yaw (Z)
        
        // Left Limelight 4 - AprilTag camera (adjust these values to your robot)
        public static final Transform3d kLimelightLeftPosition = new Transform3d(
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

        // Right Limelight 4 - AprilTag camera (adjust these values to your robot)
        public static final Transform3d kLimelightRightPosition = new Transform3d(
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

    public static class Intake {
        // Operational constants
        public static final double kSpinnerSpeed = 10; // Target spinner speed in rotations per second (RPS)
        public static final double kIntakeAngleDeployed = 45; // Deployed angle in degrees
        public static final double kIntakeAngleStowed = 0; // Stowed angle in degrees
        
        // Pivot Constants
        public static final int kPivotCanID = 10; // CAN ID (dimensionless)
        public static final double kPivotGearRatio = 9; // Gear ratio (dimensionless)
        public static final double kPivotKP = 5.07; // Proportional gain (dimensionless)
        public static final double kPivotKI = 0; // Integral gain (dimensionless)
        public static final double kPivotKD = 2.82; // Derivative gain (dimensionless)
        public static final double kPivotKS = 0; // Static friction feedforward (volts)
        public static final double kPivotKV = 1.12; // Velocity feedforward (volt-seconds per radian)
        public static final double kPivotKA = 0.08; // Acceleration feedforward (volt-seconds² per radian)
        public static final double kPivotKG = 0.53; // Gravity feedforward (volts)
        public static final double kPivotMaxVelocity = 1; // Maximum velocity (rad/s)
        public static final double kPivotMaxAcceleration = 1; // Maximum acceleration (rad/s²)
        public static final boolean kPivotBrakeEnabled = true; // Brake mode enabled (boolean)
        public static final double kPivotForwardSoftLimit = 60; // Maximum angle (degrees)
        public static final double kPivotReverseSoftLimit = 0; // Minimum angle (degrees)
        public static final boolean kPivotStatorLimitEnabled = true; // Stator current limit enabled (boolean)
        public static final double kPivotStatorCurrentLimit = 40; // Stator current limit (amperes)
        public static final boolean kPivotSupplyLimitEnabled = false; // Supply current limit enabled (boolean)
        public static final double kPivotSupplyCurrentLimit = 40; // Supply current limit (amperes)

        // Spinner constants
        public static final int kSpinnerCanID = 9; // CAN ID (dimensionless)
        public static final double kSpinnerGearRatio = 2; // Gear ratio (dimensionless)
        public static final double kSpinnerKP = 0.6; // Proportional gain (dimensionless) // 0.6 works fine for 1:2
        public static final double kSpinnerKI = 0; // Integral gain (dimensionless)
        public static final double kSpinnerKD = 0; // Derivative gain (dimensionless)
        public static final double kSpinnerKS = 0; // Static friction feedforward (volts)
        public static final double kSpinnerKV = 0; // Velocity feedforward (volt-seconds per radian)
        public static final double kSpinnerKA = 0; // Acceleration feedforward (volt-seconds² per radian)
        public static final double kSpinnerKG = 0; // Gravity feedforward (volts)
        public static final double kSpinnerMaxVelocity = 1; // Maximum velocity (rad/s)
        public static final double kSpinnerMaxAcceleration = 1; // Maximum acceleration (rad/s²)
        public static final boolean kSpinnerBrakeMode = true; // Brake mode enabled (boolean)
        public static final double kSpinnerForwardSoftLimit = 0; // Maximum angle (degrees)
        public static final double kSpinnerReverseSoftLimit = 0; // Minimum angle (degrees)
        public static final boolean kSpinnerEnableStatorLimit = true; // Stator current limit enabled (boolean)
        public static final int kSpinnerStatorCurrentLimit = 40; // Stator current limit (amperes)
        public static final boolean kSpinnerEnableSupplyLimit = false; // Supply current limit enabled (boolean)
        public static final double kSpinnerSupplyCurrentLimit = 40; // Supply current limit (amperes)
        
        // Simulation constants
        public static final double kSimArmMomentOfInertia = 0.01; // Arm moment of inertia
        public static final double kSimArmLength = 0.1; // Arm length (m)
        public static final double kSimMinAngleDegrees = -90; // Min angle (degrees)
        public static final double kSimMaxAngleDegrees = 90; // Max angle (degrees)
        public static final boolean kSimulateGravity = false; // Simulate gravity
        public static final double kSimStartingPositionDegrees = 0; // Starting position (degrees)
        
        // Spinner simulation constants
        public static final double kSimSpinnerMomentOfInertia = 0.0001; // Spinner moment of inertia (kg·m²)
    }

    public static class Spindexer {
        // Operational constants
        public static final double kSpinnerSpeed = -25; // Target spinner speed in rotations per second (RPS)
        
        // Motor Constants
        public static final int kMotorCanID = 11; // CAN ID (dimensionless)
        public static final double kGearRatio = 1; // Gear ratio (dimensionless)
        public static final double kKP = 0.6; // Proportional gain (dimensionless)
        public static final double kKI = 0; // Integral gain (dimensionless)
        public static final double kKD = 0; // Derivative gain (dimensionless)
        public static final double kKS = 0.1; // Static friction feedforward (volts)
        public static final double kKV = 0.12; // Velocity feedforward (volt-seconds per radian)
        public static final double kKA = 0.01; // Acceleration feedforward (volt-seconds² per radian)
        public static final double kMaxVelocity = 50; // Maximum velocity (rotations/s)
        public static final double kMaxAcceleration = 100; // Maximum acceleration (rotations/s²)
        public static final boolean kBrakeMode = false; // Brake mode enabled (boolean)
        public static final boolean kEnableStatorLimit = true; // Stator current limit enabled (boolean)
        public static final int kStatorCurrentLimit = 40; // Stator current limit (amperes)
        public static final boolean kEnableSupplyLimit = false; // Supply current limit enabled (boolean)
        public static final double kSupplyCurrentLimit = 40; // Supply current limit (amperes)
    }

    public static class Feeder {
        // Operational constants
        public static final double kSpinnerSpeed = 20; // Target spinner speed in rotations per second (RPS)
        
        // Motor Constants
        public static final int kMotorCanID = 12; // CAN ID (dimensionless)
        public static final double kGearRatio = 1; // Gear ratio (dimensionless)
        public static final double kKP = 0.6; // Proportional gain (dimensionless)
        public static final double kKI = 0; // Integral gain (dimensionless)
        public static final double kKD = 0; // Derivative gain (dimensionless)
        public static final double kKS = 0.1; // Static friction feedforward (volts)
        public static final double kKV = 0.12; // Velocity feedforward (volt-seconds per radian)
        public static final double kKA = 0.01; // Acceleration feedforward (volt-seconds² per radian)
        public static final double kMaxVelocity = 60; // Maximum velocity (rotations/s)
        public static final double kMaxAcceleration = 120; // Maximum acceleration (rotations/s²)
        public static final boolean kBrakeMode = false; // Brake mode enabled (boolean)
        public static final boolean kEnableStatorLimit = true; // Stator current limit enabled (boolean)
        public static final int kStatorCurrentLimit = 40; // Stator current limit (amperes)
        public static final boolean kEnableSupplyLimit = false; // Supply current limit enabled (boolean)
        public static final double kSupplyCurrentLimit = 40; // Supply current limit (amperes)
    }

    public static class Turret {

        public static final int kTurretCanID = 14; // CAN ID (dimensionless) - Changed from 11 to 12

        public static final double kTurretOffsetX  = 0.1; // X offset (forward +X, backward -X) from robot center to turret (meters) 
        public static final double kTurretOffsetY = -0.2; // Y offset (left +Y, right -Y) from robot center to turret (meters)

        // Turret mechanical limits (degrees, relative to robot forward)
        public static final double kMinAngleDegrees = -180.0;
        public static final double kMaxAngleDegrees = 180.0;

        // Shooting range limits (meters)
        public static final double kMinShootingDistance = 1.0;
        public static final double kMaxShootingDistance = 5.5;
    }
}
