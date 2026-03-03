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
        public static final double kSYOMDriveMinVelocity = 0.75; // Minimum translation speed (m/s) before auto-rotation activates
        public static final double kSYOMDriveRotationKp = 7.0; // Proportional gain for rotation to travel direction
        public static final double kSYOMDriveMaxRotationalVelocity = Units.degreesToRadians(210); // Max rotational velocity (rad/s)

        // Motion-profile constraints for SYOM rotation (TrapezoidProfile)
        public static final double kSYOMDriveMaxRotAccel = Units.degreesToRadians(360); // Max rotational accel (rad/s²)
        // Velocity slew: rotational output approaches target at this rate (rad/s per loop)
        // Set to max-velocity / desired-ramp-time  (e.g. 180°/s over 0.15 s ≈ 1200 °/s²)
        public static final double kSYOMDriveSlewRateRadPerSec = Units.degreesToRadians(400); // rate limiter (rad/s²)

        // ==================== SET-YAW / HEADING LOCK CONSTANTS ====================
        // Used by SetYawCommand with CTRE's FieldCentricFacingAngle request.
        // The request feeds these gains into its built-in PhoenixPIDController which
        // runs a standard PID + MotionMagicExpo profile on the robot heading.

        // Heading PID gains (units: rotations → rotations/s output)
        public static final double kHeadingKP = 7.0;   // Proportional gain  (tune on robot)
        public static final double kHeadingKI = 0.0;   // Integral gain
        public static final double kHeadingKD = 0.3;   // Derivative gain

        // Heading MotionMagic Expo cruise constraints
        // Cruise velocity in rotations per second (≈ 540 °/s)
        public static final double kHeadingMaxVelocityRps   = 1.5;
        // Expo feedforward coefficient (rotations/s per √(rotation)) – shapes the
        // acceleration curve; higher = snappier acceleration
        public static final double kHeadingExpoKV = 0.1;
        public static final double kHeadingExpoKA = 0.005;

        // Tolerance: command finishes when heading error is within this value
        public static final double kHeadingToleranceDegrees = 2.0;

        // Translation deadband applied to the FacingAngle request (fraction of MaxSpeed)
        // Matches the 10 % deadband used by the default field-centric drive command
        public static final double kFacingAngleTranslationDeadband = 0.1; // 10 %
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
        public static final String kLimelightLeftName = "limelight-left"; //left and two
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

        // Gamepiece Limelight 2 + Google Coral — looking down at floor from front-top of robot
        // Position: 0.19685 m forward, 0.04445 m right, 0.5147818 m up from robot center (floor)
        // Angle: 20° below horizontal (pitch = -20°), mounted UPSIDE DOWN (roll = 180°), yaw = 0°
        // NOTE: Also set "Camera Rotation = 180°" in the Limelight web interface so the
        //       firmware flips the image — txnc/tync will then be reported correctly.
        public static final Transform3d kLimelightGamepiecePosition = new Transform3d(
            new Translation3d(
                0.19685,    // X: forward (meters)
                0.04445,    // Y: right   (meters) — positive Y is right in robot space
                0.5147818   // Z: up      (meters)
            ),
            new Rotation3d(
                Units.degreesToRadians(180.0),  // Roll: 180° — camera is physically upside down
                Units.degreesToRadians(-20.0),  // Pitch: -20° (angled down toward floor)
                0.0                             // Yaw:   0° (facing straight forward)
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
        public static final double kMaxTagDistance = 10;
        
        // Maximum ambiguity to accept AprilTag readings (0-1, lower is better)
        public static final double kMaxAmbiguity = 0.3;
        
        // Minimum tag area to accept readings (percentage of image)
        public static final double kMinTagArea = 0.02;

        // ==================== MEGATAG2 CONFIG ====================
        // Whether to use MegaTag2 (recommended for better accuracy)
        public static final boolean kUseMegaTag2 = false;

        // ==================== CAMERA POSE CONFIG ====================
        // Set to true to override web GUI camera poses with code values
        // Set to false to use values configured in Limelight web interface
        public static final boolean kSetCameraPosesFromCode = false;

        // ==================== GAMEPIECE DETECTION CONFIG ====================
        // Class ID assigned to "Fuel" (yellow ball) by the trained Coral model
        public static final int kFuelClassId = 0;

        // Minimum detection area (% of image) to accept a ball detection — filters distant/noisy hits
        public static final double kMinDetectionArea = 0.05;

        // Horizontal field-of-view of the Limelight 2 (degrees) — used for 3D projection
        public static final double kGamepieceCameraHFovDeg = 62.5;

        // Vertical field-of-view of the Limelight 2 (degrees)
        public static final double kGamepieceCameraVFovDeg = 48.9;

        // Diameter of a Fuel ball (meters)
        public static final double kBallDiameterMeters = 0.15;

        // ==================== WHISKER PICKUP ALGORITHM CONFIG ====================
        // Width of the intake opening (meters) — determines how wide each whisker is
        public static final double kIntakeWidthMeters = 0.6096;

        // How far ahead each whisker projects to score balls (meters)
        public static final double kWhiskerLengthMeters = 3.0;

        // Angular resolution of the whisker fan — one whisker every N degrees
        // Smaller = finer resolution but slightly more CPU; 5° is a good balance
        public static final double kWhiskerAngleStepDeg = 5.0;

        // Half-angle of the forward fan — whiskers sweep ±kWhiskerFanHalfAngleDeg from robot forward
        // 60° gives a ±60° (120° total) forward cone
        public static final double kWhiskerFanHalfAngleDeg = 60.0;

        // Value awarded per ball captured by a whisker path (points)
        public static final double kWhiskerBallValue = 1.0;

        // Drive speed used during auto ball pickup (m/s)
        public static final double kPickupDriveSpeed = 1.5;

        // Rotation P-gain used during whisker pickup to align robot heading to best whisker angle
        public static final double kPickupRotationKp = 5.0;

        // Maximum rotational rate allowed during pickup (rad/s)
        public static final double kPickupMaxRotationalRateRadS = Math.toRadians(180.0);

        // Minimum score a best whisker must have for the robot to chase it; prevents chasing nothing
        public static final double kPickupMinScoreThreshold = 0.5;
    }

    public static class Intake {
        // Operational constants
        public static final double kSpinnerSpeed = -28; // Target spinner speed in rotations per second (RPS)
        public static final double kIntakeAngleDeployed = -55; // Deployed angle in degrees
        public static final double kIntakeAngleStowed = 0; // Stowed angle in degrees
        public static final double kPivotRezeroAngleDegrees = 0.0; // Rezero position in degrees
        
        // Pivot Constants
        public static final int kPivotCanID = 10; // CAN ID (dimensionless)
        public static final double kPivotGearRatio = 9; // Gear ratio (dimensionless)
        public static final double kPivotKP = 500; // Proportional gain (dimensionless)
        public static final double kPivotKI = 0; // Integral gain (dimensionless)
        public static final double kPivotKD = 0; // Derivative gain (dimensionless)
        public static final double kPivotKS = 0; // Static friction feedforward (volts)
        public static final double kPivotKV = 20; // Velocity feedforward (volt-seconds per radian)
        public static final double kPivotKA = 0.08; // Acceleration feedforward (volt-seconds² per radian)
        public static final double kPivotKG = 0; // Gravity feedforward (volts)
        public static final double kPivotMaxVelocity = 1; // Maximum velocity for Motion Magic (rotations/s, mechanism-side)
        public static final double kPivotMaxAcceleration = 2.0; // Maximum acceleration for Motion Magic (rotations/s², mechanism-side)
        public static final boolean kPivotBrakeEnabled = true; // Brake mode enabled (boolean)
        public static final double kPivotForwardSoftLimit = 100; // Maximum angle (degrees)
        public static final double kPivotReverseSoftLimit = -100; // Minimum angle (degrees)
        public static final boolean kPivotStatorLimitEnabled = true; // Stator current limit enabled (boolean)
        public static final double kPivotStatorCurrentLimit = 100; // Stator current limit (amperes)
        public static final boolean kPivotSupplyLimitEnabled = true; // Supply current limit enabled (boolean)
        public static final double kPivotSupplyCurrentLimit = 30; // Supply current limit (amperes)

        // Spinner constants
        public static final int kSpinnerCanID = 9; // CAN ID (dimensionless)
        public static final double kSpinnerGearRatio = 2; // Gear ratio (dimensionless)
        public static final double kSpinnerKP = 16; // Proportional gain (dimensionless) // 0.6 works fine for 1:2
        public static final double kSpinnerKI = 0; // Integral gain (dimensionless)
        public static final double kSpinnerKD = 0; // Derivative gain (dimensionless)
        public static final double kSpinnerKS = 0; // Static friction feedforward (volts)
        public static final double kSpinnerKV = 0.235; // Velocity feedforward (volt-seconds per radian)
        public static final double kSpinnerKA = 0; // Acceleration feedforward (volt-seconds² per radian)
        public static final double kSpinnerKG = 0; // Gravity feedforward (volts)
        public static final double kSpinnerMaxVelocity = 40; // Maximum velocity (rad/s)
        public static final double kSpinnerMaxAcceleration = 100; // Maximum acceleration (rad/s²)
        public static final boolean kSpinnerBrakeMode = true; // Brake mode enabled (boolean)
        public static final double kSpinnerForwardSoftLimit = 0; // Maximum angle (degrees)
        public static final double kSpinnerReverseSoftLimit = 0; // Minimum angle (degrees)
        public static final boolean kSpinnerEnableStatorLimit = true; // Stator current limit enabled (boolean)
        public static final int kSpinnerStatorCurrentLimit = 30; // Stator current limit (amperes)
        public static final boolean kSpinnerEnableSupplyLimit = true; // Supply current limit enabled (boolean)
        public static final double kSpinnerSupplyCurrentLimit = 25; // Supply current limit (amperes)
        
        // Simulation constants
        public static final double kSimArmMomentOfInertia = 0.3; // Arm moment of inertia (kg·m²)
        public static final double kSimArmLength = 0.4; // Arm length (m) - ~16 inches
        public static final double kSimMinAngleDegrees = -60; // Min angle (degrees)
        public static final double kSimMaxAngleDegrees = 30; // Max angle (degrees)
        public static final boolean kSimulateGravity = true; // Simulate gravity
        public static final double kSimStartingPositionDegrees = 0; // Starting position (degrees)
        
        // Spinner simulation constants
        public static final double kSimSpinnerMomentOfInertia = 0.0001; // Spinner moment of inertia (kg·m²)
    }

    public static class Spindexer {
        // Operational constants
        public static final double kSpinnerSpeed = -1.8; // Target spinner speed in rotations per second (RPS)
        
        // Motor Constants
        public static final int kMotorCanID = 11; // CAN ID (dimensionless)
        public static final double kGearRatio = 15; // Gear ratio (dimensionless)
        public static final double kKP = 11; // Proportional gain (dimensionless)
        public static final double kKI = 0; // Integral gain (dimensionless)
        public static final double kKD = 0; // Derivative gain (dimensionless)
        public static final double kKS = 0; // Static friction feedforward (volts)
        public static final double kKV = 1.9; // Velocity feedforward (volt-seconds per radian)
        public static final double kKA = 1; // Acceleration feedforward (volt-seconds² per radian)
        public static final double kMaxVelocity = 50; // Maximum velocity (rotations/s)
        public static final double kMaxAcceleration = 100; // Maximum acceleration (rotations/s²)
        public static final boolean kBrakeMode = true; // Brake mode enabled (boolean)
        public static final boolean kEnableStatorLimit = true; // Stator current limit enabled (boolean)
        public static final int kStatorCurrentLimit = 40; // Stator current limit (amperes)
        public static final boolean kEnableSupplyLimit = false; // Supply current limit enabled (boolean)
        public static final double kSupplyCurrentLimit = 40; // Supply current limit (amperes)
    }

    public static class Feeder {
        // Operational constants
        public static final double kSpinnerSpeed = -25; // Target spinner speed in rotations per second (RPS)
        
        // Motor Constants
        public static final int kMotorCanID = 12; // CAN ID (dimensionless)
        public static final double kGearRatio = 1; // Gear ratio (dimensionless)
        public static final double kKP = 10; // Proportional gain (dimensionless)
        public static final double kKI = 0; // Integral gain (dimensionless)
        public static final double kKD = 0; // Derivative gain (dimensionless)
        public static final double kKS = 0; // Static friction feedforward (volts)
        public static final double kKV = 0.12; // Velocity feedforward (volt-seconds per radian)
        public static final double kKA = 1; // Acceleration feedforward (volt-seconds² per radian)
        public static final double kMaxVelocity = 60; // Maximum velocity (rotations/s)
        public static final double kMaxAcceleration = 120; // Maximum acceleration (rotations/s²)
        public static final boolean kBrakeMode = true; // Brake mode enabled (boolean)
        public static final boolean kEnableStatorLimit = true; // Stator current limit enabled (boolean)
        public static final int kStatorCurrentLimit = 40; // Stator current limit (amperes)
        public static final boolean kEnableSupplyLimit = false; // Supply current limit enabled (boolean)
        public static final double kSupplyCurrentLimit = 30; // Supply current limit (amperes)
    }

    public static class Turret {

        public static final int kTurretCanID = 13; // CAN ID (dimensionless) - Changed to 13 since shooter uses 14-15

        public static final double kTurretOffsetX  = -0.2; // X offset (forward +X, backward -X) from robot center to turret (meters) 
        public static final double kTurretOffsetY = -0.2; // Y offset (left +Y, right -Y) from robot center to turret (meters)
        public static final double kTurretOffsetZ = 0.3; // Z offset (up +Z, down -Z) from robot center to turret (meters)
        public static final double kTurretStartAngleDegrees = 0.1; // Physical starting angle (encoder will be set to this value at init)
        public static final double kTurretRezeroAngleDegrees = 0.0; // Angle to set encoder to when rezeroing (0 = forward)

        // Turret mechanical limits (degrees, relative to robot forward)
        public static final double kMinAngleDegrees = -200.0;
        public static final double kMaxAngleDegrees = 180.0;

        // Shooting range limits (meters)
        public static final double kMinShootingDistance = 0.5;
        public static final double kMaxShootingDistance = 10.0;

        // Motor and control constants
        public static final double kGearRatio = 11.111; // Gear ratio (dimensionless)
        public static final double kKP = 140; // Proportional gain (dimensionless)
        public static final double kKI = 0.01; // Integral gain (dimensionless)
        public static final double kKD = 0; // Derivative gain (dimensionless)
        public static final double kKS = 0; // Static friction feedforward (volts)
        public static final double kKV = 1; // Velocity feedforward (volt-seconds per radian)
        public static final double kKA = 0; // Acceleration feedforward (volt-seconds² per radian)
        public static final double kKG = 0; // Gravity feedforward (volts) - Unused for turrets
        public static final double kMaxVelocity = 1.0; // Maximum velocity for Motion Magic (rotations/s, mechanism-side)
        public static final double kMaxAcceleration = 2.0; // Maximum acceleration for Motion Magic (rotations/s², mechanism-side)
        public static final boolean kBrakeMode = true; // Brake mode enabled (boolean)
        public static final boolean kEnableStatorLimit = true; // Stator current limit enabled (boolean)
        public static final double kStatorCurrentLimit = 40; // Stator current limit (amperes)
        public static final boolean kEnableSupplyLimit = false; // Supply current limit enabled (boolean)
        public static final double kSupplyCurrentLimit = 40; // Supply current limit (amperes)
    }

     public static class Shooter {
        // Flywheel Constants
        public static final int kLeftFlywheelCanID = 14; // CAN ID for left flywheel
        public static final int kRightFlywheelCanID = 15; // CAN ID for right flywheel
        public static final double kFlywheelGearRatio = 2.0; // 2:1 gear ratio
        public static final double kDefaultFlywheelSpeed = -1.0; // Default speed in RPS
        
        // Flywheel PID Constants
        public static final double kFlywheelKP = 0.24;
        public static final double kFlywheelKI = 0;
        public static final double kFlywheelKD = 0;
        public static final double kFlywheelKS = 0.1;
        public static final double kFlywheelKV = 0.235;
        public static final double kFlywheelKA = 2.32;
        
        // Flywheel Current Limits
        public static final boolean kFlywheelEnableStatorLimit = true;
        public static final int kFlywheelStatorCurrentLimit = 60; // Higher for flywheels
        public static final boolean kFlywheelEnableSupplyLimit = false;
        public static final double kFlywheelSupplyCurrentLimit = 60;
        public static final boolean kFlywheelBrakeMode = false; // Coast mode for flywheels
        
        // Hood Constants
        public static final int kHoodCanID = 1; // CAN ID for hood SparkMax
        public static final double kHoodGearRatio =115.3476; // experimentall determined gear ratio (motor rotations per hood rotation)
        public static final double kHoodMinAngleDegrees = 45.0; // Minimum hood angle
        public static final double kHoodMaxAngleDegrees = 80.0; // Maximum hood angle (shooting straight up)
        public static final double kHoodStartAngleDegrees = 80.0; // Starting position
        public static final double kHoodRezeroAngleDegrees = 80.0; // Rezero position in degrees
        public static final double kHoodTrenchAngleDegrees = 80.0; // Trench position in degrees
        
        // Hood PID Constants
        public static final double kHoodKP =  1;
        public static final double kHoodKI = 0.01;
        public static final double kHoodKD = 0;
        public static final double kHoodKS = 0;
        public static final double kHoodKV = 0;
        public static final double kHoodKA = 0;

        // Flywheel at-speed tolerance
        public static final double kFlywheelAtSpeedToleranceRPS = 1.0; // ±1 RPS triggers "at speed" rumble

        // Hood MAXMotion (motion profiling) constraints
        public static final double kHoodMaxVelocity = 300.0;      // Maximum hood velocity (degrees/s)
        public static final double kHoodMaxAcceleration = 300.0; // Maximum hood acceleration (degrees/s²)

        // Hood Current Limits
        public static final int kHoodCurrentLimit = 20; // Amps for NEO 550
        public static final boolean kHoodBrakeMode = true;
        
        // Simulation Constants
        public static final double kSimFlywheelMomentOfInertia = 0.005; // kg·m² — realistic for shooter flywheels
        public static final double kSimHoodMomentOfInertia = 0.004; // kg·m²
        public static final double kSimHoodLength = 0.3; // meters
    }
}
