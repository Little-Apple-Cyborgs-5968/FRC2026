// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

/**
 * Vision subsystem that handles AprilTag-based robot localization using multiple Limelights.
 * 
 * This subsystem manages:
 * - Two Limelight 4 cameras for AprilTag detection and MegaTag2 pose estimation (left and right)
 * - One Limelight 2 with Google Coral for gamepiece detection (placeholder)
 * 
 * Pose estimates are fed to the drivetrain's pose estimator for fusion with odometry.
 * All vision data is logged to NetworkTables for AdvantageScope visualization.
 * AprilTag field positions are loaded from a .fmap file for accurate tag pose logging.
 */
public class Vision extends SubsystemBase {
    
    // ==================== APRILTAG FIELD MAP ====================
    private final Map<Integer, Pose3d> aprilTagFieldMap = new HashMap<>();
    
    // ==================== DRIVETRAIN INTERFACE ====================
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<Rotation2d> headingSupplier;
    private final VisionConsumer visionConsumer;

    // ==================== NETWORKTABLES LOGGING ====================
    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    
    // Estimated poses from each Limelight (for AdvantageScope)
    private final StructPublisher<Pose2d> leftPosePublisher;
    private final StructPublisher<Pose2d> rightPosePublisher;
    private final StructPublisher<Pose2d> backPosePublisher;
    
    // Detected AprilTag positions (for AdvantageScope)
    private final StructArrayPublisher<Pose3d> leftTagsPublisher;
    private final StructArrayPublisher<Pose3d> rightTagsPublisher;
    private final StructArrayPublisher<Pose3d> backTagsPublisher;

    // Detected ball positions relative to robot, published as flat [x0,y0, x1,y1, ...] array
    private final DoubleArrayPublisher ballPositionsPublisher;

    // ==================== LATEST ESTIMATES ====================
    private PoseEstimate latestLeftEstimate = null;
    private PoseEstimate latestRightEstimate = null;
    private PoseEstimate latestBackEstimate = null;

    // ==================== LATEST BALL DATA ====================
    /** Latest ball positions in robot-relative space (updated every periodic). */
    private List<Translation2d> latestBallPositions = new ArrayList<>();

    /**
     * Functional interface for adding vision measurements to the drivetrain.
     */
    @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
    }

    /**
     * Creates a new Vision subsystem.
     * 
     * @param poseSupplier Supplier for the current robot pose from odometry
     * @param headingSupplier Supplier for the current robot heading (for MegaTag2)
     * @param visionConsumer Consumer that feeds vision measurements to the drivetrain
     */
    public Vision(
        Supplier<Pose2d> poseSupplier,
        Supplier<Rotation2d> headingSupplier,
        VisionConsumer visionConsumer
    ) {
        this.poseSupplier = poseSupplier;
        this.headingSupplier = headingSupplier;
        this.visionConsumer = visionConsumer;

        // Initialize NetworkTables publishers for logging
        var visionTable = ntInstance.getTable("VISION");
        
        leftPosePublisher = visionTable.getStructTopic("LeftEstimatedPose", Pose2d.struct).publish();
        rightPosePublisher = visionTable.getStructTopic("RightEstimatedPose", Pose2d.struct).publish();
        backPosePublisher = visionTable.getStructTopic("BackEstimatedPose", Pose2d.struct).publish();
        
        leftTagsPublisher = visionTable.getStructArrayTopic("LeftDetectedTags", Pose3d.struct).publish();
        rightTagsPublisher = visionTable.getStructArrayTopic("RightDetectedTags", Pose3d.struct).publish();
        backTagsPublisher = visionTable.getStructArrayTopic("BackDetectedTags", Pose3d.struct).publish();
        ballPositionsPublisher = visionTable.getDoubleArrayTopic("BallPositions").publish();

        // Load AprilTag field map from .fmap file
        loadAprilTagFieldMap();

        // Configure Limelights on startup
        configureLimelights();
    }

    /**
     * Loads AprilTag positions from a .fmap file in the deploy directory.
     * The .fmap file is a JSON file used by Limelight and PathPlanner for field layout.
     * 
     * Expected file location: deploy/2026-reefscape.fmap
     * 
     * The .fmap format contains "fiducials" array with objects containing:
     * - "id": AprilTag ID (integer)
     * - "transform": [x, y, z, qw, qx, qy, qz] (position in meters, rotation as quaternion)
     */
    private void loadAprilTagFieldMap() {
        try {
            File fmapFile = new File(Filesystem.getDeployDirectory(), Constants.Vision.kFieldMapFileName);
            
            if (!fmapFile.exists()) {
                DriverStation.reportWarning(
                    "AprilTag field map not found: " + fmapFile.getAbsolutePath() + 
                    ". Tag positions will not be logged to AdvantageScope.", false);
                return;
            }

            ObjectMapper mapper = new ObjectMapper();
            JsonNode root = mapper.readTree(fmapFile);
            JsonNode fiducials = root.get("fiducials");

            if (fiducials == null || !fiducials.isArray()) {
                DriverStation.reportWarning("Invalid .fmap file format: missing 'fiducials' array", false);
                return;
            }

            for (JsonNode fiducial : fiducials) {
                int id = fiducial.get("id").asInt();
                JsonNode transform = fiducial.get("transform");

                if (transform != null && transform.isArray() && transform.size() >= 16) {
                    // The .fmap transform is a 4x4 transformation matrix stored row-major:
                    // [r00, r01, r02, tx, r10, r11, r12, ty, r20, r21, r22, tz, 0, 0, 0, 1]
                    // Extract position from the 4th column (indices 3, 7, 11)
                    double x = transform.get(3).asDouble();
                    double y = transform.get(7).asDouble();
                    double z = transform.get(11).asDouble();

                    // Extract rotation matrix elements
                    double r00 = transform.get(0).asDouble();
                    double r01 = transform.get(1).asDouble();
                    double r02 = transform.get(2).asDouble();
                    double r10 = transform.get(4).asDouble();
                    double r11 = transform.get(5).asDouble();
                    double r12 = transform.get(6).asDouble();
                    double r20 = transform.get(8).asDouble();
                    double r21 = transform.get(9).asDouble();
                    double r22 = transform.get(10).asDouble();

                    // Create Rotation3d from rotation matrix using Matrix
                    Matrix<N3, N3> rotMatrix = new Matrix<>(edu.wpi.first.math.Nat.N3(), edu.wpi.first.math.Nat.N3());
                    rotMatrix.set(0, 0, r00);
                    rotMatrix.set(0, 1, r01);
                    rotMatrix.set(0, 2, r02);
                    rotMatrix.set(1, 0, r10);
                    rotMatrix.set(1, 1, r11);
                    rotMatrix.set(1, 2, r12);
                    rotMatrix.set(2, 0, r20);
                    rotMatrix.set(2, 1, r21);
                    rotMatrix.set(2, 2, r22);
                    
                    Rotation3d rotation = new Rotation3d(rotMatrix);
                    Translation3d translation = new Translation3d(x, y, z);
                    Pose3d tagPose = new Pose3d(translation, rotation);

                    aprilTagFieldMap.put(id, tagPose);
                }
            }

            System.out.println("[Vision] Loaded " + aprilTagFieldMap.size() + " AprilTags from " + Constants.Vision.kFieldMapFileName);

        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTag field map: " + e.getMessage(), e.getStackTrace());
        }
    }

    /**
     * Configures all Limelights with proper pipeline and LED settings.
     */
    private void configureLimelights() {
        // Set AprilTag Limelights to pipeline 0 (should be AprilTag pipeline)
        LimelightHelpers.setPipelineIndex(Constants.Vision.kLimelightLeftName, 0);
        LimelightHelpers.setPipelineIndex(Constants.Vision.kLimelightRightName, 0);
        
        // Set gamepiece Limelight to neural network pipeline
        LimelightHelpers.setPipelineIndex(Constants.Vision.kLimelightBackeName, 0);
        
        // Turn off LEDs for AprilTag cameras (they don't need them)
        LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.kLimelightLeftName);
        LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.kLimelightRightName);
        LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.kLimelightBackeName);
        
        // Configure camera positions relative to robot center using Transform3d constants
        // These override the web GUI values - comment out to use GUI values instead
        if (Constants.Vision.kSetCameraPosesFromCode) {
            setCameraPoseFromTransform(Constants.Vision.kLimelightLeftName, Constants.Vision.kLimelightLeftPosition);
            setCameraPoseFromTransform(Constants.Vision.kLimelightRightName, Constants.Vision.kLimelightRightPosition);
            setCameraPoseFromTransform(Constants.Vision.kLimelightBackeName, Constants.Vision.kLimelightGamepiecePosition);
        }
    }

    /**
     * Sets the camera pose on a Limelight from a Transform3d.
     * 
     * @param limelightName The NetworkTables name of the Limelight
     * @param transform The Transform3d representing the camera position relative to robot center
     */
    private void setCameraPoseFromTransform(String limelightName, Transform3d transform) {
        LimelightHelpers.setCameraPose_RobotSpace(
            limelightName,
            transform.getX(),                                    // Forward (meters)
            transform.getY(),                                    // Side (meters)
            transform.getZ(),                                    // Up (meters)
            Math.toDegrees(transform.getRotation().getX()),      // Roll (degrees)
            Math.toDegrees(transform.getRotation().getY()),      // Pitch (degrees)
            Math.toDegrees(transform.getRotation().getZ())       // Yaw (degrees)
        );
    }

    @Override
    public void periodic() {
        // Update robot orientation for MegaTag2 on both cameras
        updateRobotOrientation();
        
        // Process AprilTag vision from both cameras
        processAprilTagVision(Constants.Vision.kLimelightLeftName, "left");
        processAprilTagVision(Constants.Vision.kLimelightRightName, "right");
        
        if (Constants.Vision.kUseBackLimelightForPose) {
            processAprilTagVision(Constants.Vision.kLimelightBackeName, "back");
        }
        
        // Log detected AprilTags to NetworkTables
        logDetectedTags();
        
        // Process and log gamepiece (Fuel ball) detections
        processGamepieceDetection();
    }

    /**
     * Updates the robot orientation on all AprilTag Limelights for MegaTag2.
     * This is required for accurate pose estimation with MegaTag2.
     */
    private void updateRobotOrientation() {
        Rotation2d heading = headingSupplier.get();
        double yawDegrees = heading.getDegrees();
        
        // Set robot orientation for MegaTag2 on AprilTag cameras
        LimelightHelpers.SetRobotOrientation(
            Constants.Vision.kLimelightLeftName,
            yawDegrees, 0, 0, 0, 0, 0
        );
        LimelightHelpers.SetRobotOrientation(
            Constants.Vision.kLimelightRightName,
            yawDegrees, 0, 0, 0, 0, 0
        );
        if (Constants.Vision.kUseBackLimelightForPose) {
            LimelightHelpers.SetRobotOrientation(
                Constants.Vision.kLimelightBackeName,
                yawDegrees, 0, 0, 0, 0, 0
            );
        }
    }

    /**
     * Processes AprilTag vision data from a single Limelight and feeds it to the pose estimator.
     * 
     * @param limelightName The NetworkTables name of the Limelight
     * @param cameraPosition The position of the camera ("left", "right", or "back")
     */
    private void processAprilTagVision(String limelightName, String cameraPosition) {
        // Get pose estimate based on configuration
        PoseEstimate estimate;
        if (Constants.Vision.kUseMegaTag2) {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        } else {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        }

        // Store for logging
        if (cameraPosition.equals("left")) {
            latestLeftEstimate = estimate;
        } else if (cameraPosition.equals("right")) {
            latestRightEstimate = estimate;
        } else {
            latestBackEstimate = estimate;
        }

        // Validate the estimate
        if (!isValidEstimate(estimate)) {
            return;
        }

        // Log the estimated pose to NetworkTables
        if (cameraPosition.equals("left")) {
            leftPosePublisher.set(estimate.pose);
        } else if (cameraPosition.equals("right")) {
            rightPosePublisher.set(estimate.pose);
        } else {
            backPosePublisher.set(estimate.pose);
        }

        // Calculate standard deviations based on number of tags
        Matrix<N3, N1> stdDevs = calculateStdDevs(estimate, cameraPosition.equals("back"));

        // Feed the vision measurement to the drivetrain
        visionConsumer.accept(estimate.pose, estimate.timestampSeconds, stdDevs);
    }

    /**
     * Validates a pose estimate based on configurable thresholds.
     * 
     * @param estimate The pose estimate to validate
     * @return true if the estimate passes all validity checks
     */
    private boolean isValidEstimate(PoseEstimate estimate) {
        // Null check
        if (estimate == null || estimate.pose == null) {
            return false;
        }

        // Must have at least one tag
        if (estimate.tagCount == 0) {
            return false;
        }

        // Skip during disabled to avoid drift issues
        if (DriverStation.isDisabled()) {
            return true; // Still update pose when disabled for initial localization
        }

        // Check average tag distance
        if (estimate.avgTagDist > Constants.Vision.kMaxTagDistance) {
            return false;
        }

        // Check tag area
        if (estimate.avgTagArea < Constants.Vision.kMinTagArea) {
            return false;
        }

        // For single-tag estimates, check ambiguity
        if (estimate.tagCount == 1 && estimate.rawFiducials != null && estimate.rawFiducials.length > 0) {
            if (estimate.rawFiducials[0].ambiguity > Constants.Vision.kMaxAmbiguity) {
                return false;
            }
        }

        // Sanity check: pose should be on the field (2026 field is ~16.5m x 8.0m)
        if (estimate.pose.getX() < -1 || estimate.pose.getX() > 18 ||
            estimate.pose.getY() < -1 || estimate.pose.getY() > 9) {
            return false;
        }

        return true;
    }

    /**
     * Calculates standard deviations for a pose estimate based on the number and quality of tags.
     * 
     * @param estimate The pose estimate
     * @param isBackCamera Whether this is from the back camera (higher variance)
     * @return Standard deviation matrix for the pose estimator
     */
    private Matrix<N3, N1> calculateStdDevs(PoseEstimate estimate, boolean isBackCamera) {
        if (estimate.tagCount >= 2) {
            // Multi-tag: more accurate
            return isBackCamera ? Constants.Vision.kBackMultiTagStdDevs : Constants.Vision.kMultiTagStdDevs;
        } else {
            // Single tag: less accurate, scale by distance
            double distanceScale = Math.max(1.0, estimate.avgTagDist / 2.0);
            Matrix<N3, N1> baseStdevs = isBackCamera ? Constants.Vision.kBackSingleTagStdDevs : Constants.Vision.kSingleTagStdDevs;
            return baseStdevs.times(distanceScale);
        }
    }

    /**
     * Logs detected AprilTag positions to NetworkTables for AdvantageScope visualization.
     */
    private void logDetectedTags() {
        // Log left camera tags
        List<Pose3d> leftTags = getDetectedTagPoses(Constants.Vision.kLimelightLeftName);
        leftTagsPublisher.set(leftTags.toArray(new Pose3d[0]));

        // Log right camera tags
        List<Pose3d> rightTags = getDetectedTagPoses(Constants.Vision.kLimelightRightName);
        rightTagsPublisher.set(rightTags.toArray(new Pose3d[0]));
        
        // Log back camera tags if enabled
        if (Constants.Vision.kUseBackLimelightForPose) {
            List<Pose3d> backTags = getDetectedTagPoses(Constants.Vision.kLimelightBackeName);
            backTagsPublisher.set(backTags.toArray(new Pose3d[0]));
        }
    }

    /**
     * Extracts the robot-relative 3D poses of detected AprilTags from a Limelight.
     * For AdvantageScope, we want robot-relative positions, not field positions.
     * 
     * @param limelightName The name of the Limelight to get fiducial data from
     * @return List of Pose3d objects representing detected tag positions relative to the robot
     */
    private List<Pose3d> getDetectedTagPoses(String limelightName) {
        List<Pose3d> tagPoses = new ArrayList<>();
        
        try {
            // Get the full results from the Limelight
            LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
            
            if (results == null || results.targets_Fiducials == null) {
                return tagPoses;
            }

            // Extract robot-relative poses from each detected fiducial
            for (LimelightHelpers.LimelightTarget_Fiducial fiducial : results.targets_Fiducials) {
                if (fiducial != null) {
                    // Get the robot-relative pose of the detected AprilTag
                    Pose3d robotRelativePose = fiducial.getTargetPose_RobotSpace();
                    if (robotRelativePose != null) {
                        tagPoses.add(robotRelativePose);
                    }
                }
            }
        } catch (Exception e) {
            // Log error but don't crash
            System.err.println("[Vision] Error getting fiducial data from " + limelightName + ": " + e.getMessage());
        }

        return tagPoses;
    }

    /**
     * Gets the number of AprilTags loaded from the field map.
     * @return The number of tags in the field map
     */
    public int getLoadedTagCount() {
        return aprilTagFieldMap.size();
    }

    // ==================== PUBLIC ACCESSOR METHODS ====================

    /**
     * Gets the latest pose estimate from the left camera.
     * @return The latest PoseEstimate, or null if none available
     */
    public PoseEstimate getLatestLeftEstimate() {
        return latestLeftEstimate;
    }

    /**
     * Gets the latest pose estimate from the right camera.
     * @return The latest PoseEstimate, or null if none available
     */
    public PoseEstimate getLatestRightEstimate() {
        return latestRightEstimate;
    }

    /**
     * Gets the latest pose estimate from the back camera.
     * @return The latest PoseEstimate, or null if none available
     */
    public PoseEstimate getLatestBackEstimate() {
        return latestBackEstimate;
    }

    /**
     * Checks if any camera currently sees AprilTags.
     * @return true if at least one tag is visible
     */
    public boolean hasAprilTagTarget() {
        boolean leftHasTarget = latestLeftEstimate != null && latestLeftEstimate.tagCount > 0;
        boolean rightHasTarget = latestRightEstimate != null && latestRightEstimate.tagCount > 0;
        boolean backHasTarget = Constants.Vision.kUseBackLimelightForPose && latestBackEstimate != null && latestBackEstimate.tagCount > 0;
        return leftHasTarget || rightHasTarget || backHasTarget;
    }

    /**
     * Gets the total number of AprilTags currently visible across all cameras.
     * @return The total tag count
     */
    public int getTotalTagCount() {
        int count = 0;
        if (latestLeftEstimate != null) count += latestLeftEstimate.tagCount;
        if (latestRightEstimate != null) count += latestRightEstimate.tagCount;
        if (Constants.Vision.kUseBackLimelightForPose && latestBackEstimate != null) count += latestBackEstimate.tagCount;
        return count;
    }

    // ==================== GAMEPIECE DETECTION ====================

    /**
     * Reads raw neural-network detections from the gamepiece Limelight, filters for
     * Fuel balls, projects each detection to a robot-relative 2D floor position, and
     * publishes the results to NetworkTables.
     *
     * <p>Coordinate maths overview:
     * <ol>
     *   <li>Limelight reports (txnc, tync) — horizontal and vertical angles from the
     *       camera optical centre to the centre of the bounding box (degrees).</li>
     *   <li>We know the camera is mounted {@code kCameraHeightMeters} above the floor
     *       and tilted {@code kCameraPitchDeg} below horizontal.</li>
     *   <li>The angle to the floor from the camera optical-centre for a pixel at vertical
     *       angle {@code tync} is: {@code floorAngle = kCameraPitchDeg + tync} (both
     *       measured downward-positive from horizontal).</li>
     *   <li>Ground distance: {@code d = cameraHeight / tan(floorAngle)}.</li>
     *   <li>Lateral offset: {@code lateral = d * tan(txnc)}.</li>
     *   <li>The camera is offset from the robot centre by the Transform3d in Constants;
     *       we add those offsets to get the ball's position in the robot frame.</li>
     * </ol>
     */
    private void processGamepieceDetection() {
        List<Translation2d> balls = new ArrayList<>();

        LimelightHelpers.RawDetection[] detections =
            LimelightHelpers.getRawDetections(Constants.Vision.kLimelightBackeName);

        for (LimelightHelpers.RawDetection det : detections) {
            // Filter: must be the correct class and large enough to be reliable
            if (det.classId != Constants.Vision.kFuelClassId) continue;
            if (det.ta < Constants.Vision.kMinDetectionArea)   continue;

            Translation2d ballRobotRelative = projectBallToFloor(det.txnc, det.tync);
            if (ballRobotRelative != null) {
                balls.add(ballRobotRelative);
            }
        }

        // Replace list atomically — only currently-seen balls are kept
        latestBallPositions = balls;

        // Publish as flat [x0, y0, x1, y1, ...] array for NetworkTables / AdvantageScope
        double[] flat = new double[balls.size() * 2];
        for (int i = 0; i < balls.size(); i++) {
            flat[i * 2]     = balls.get(i).getX();
            flat[i * 2 + 1] = balls.get(i).getY();
        }
        ballPositionsPublisher.set(flat);
    }

    /**
     * Projects a single Limelight detection (horizontal/vertical pixel angles) to a
     * robot-relative floor position using the known camera geometry.
     *
     * @param txnc Horizontal angle to target from image centre (degrees, +left)
     * @param tync Vertical angle to target from image centre (degrees, +up in LL convention)
     * @return Robot-relative Translation2d (X = forward, Y = left), or {@code null} if
     *         the geometry gives a physically impossible result (e.g. target is above horizon)
     */
    private Translation2d projectBallToFloor(double txnc, double tync) {
        // Camera mount geometry from Constants
        double cameraHeightM  = Constants.Vision.kLimelightGamepiecePosition.getZ();   // metres above floor
        double cameraPitchDeg = -Math.toDegrees(
            Constants.Vision.kLimelightGamepiecePosition.getRotation().getY());        // positive = nose-down
        double cameraForwardOffsetM = Constants.Vision.kLimelightGamepiecePosition.getX();
        double cameraLateralOffsetM = Constants.Vision.kLimelightGamepiecePosition.getY();

        // Limelight tync is positive-UP; convert to positive-DOWN for our geometry
        double tyncDown = -tync;

        // Total depression angle from horizontal to the detected pixel
        double totalDepressionDeg = cameraPitchDeg + tyncDown;

        // Guard: if total depression ≤ 0 the ray points above or along the horizon — reject
        if (totalDepressionDeg <= 0.0) return null;

        double totalDepressionRad = Math.toRadians(totalDepressionDeg);
        double txncRad            = Math.toRadians(txnc);

        // Horizontal (forward) distance from the camera to the ball contact point on the floor
        double forwardDistFromCamera = cameraHeightM / Math.tan(totalDepressionRad);

        // Lateral offset from the camera centre line (positive = left in robot frame)
        double lateralDistFromCamera = forwardDistFromCamera * Math.tan(txncRad);

        // Translate from camera frame to robot centre frame
        // Camera +X is robot forward; camera +Y is robot right, so we negate for robot +Y (left)
        double xRobot = forwardDistFromCamera + cameraForwardOffsetM;
        double yRobot = -lateralDistFromCamera + (-cameraLateralOffsetM); // negate both: LL +right, robot +left

        return new Translation2d(xRobot, yRobot);
    }

    // ==================== PUBLIC GAMEPIECE ACCESSOR METHODS ====================

    /**
     * Returns whether at least one Fuel ball is currently detected.
     *
     * @return {@code true} if one or more balls are visible
     */
    public boolean hasGamepieceTarget() {
        return !latestBallPositions.isEmpty();
    }

    /**
     * Returns a snapshot of all currently-detected ball positions in robot-relative
     * space (X = forward, Y = left, both in metres).
     *
     * @return Unmodifiable list of {@link Translation2d} ball positions; empty if none seen
     */
    public List<Translation2d> getBallPositions() {
        return List.copyOf(latestBallPositions);
    }

    /**
     * Returns the robot-relative position of the closest detected ball, or
     * {@code null} if no balls are visible.
     *
     * @return Closest ball {@link Translation2d}, or {@code null}
     */
    public Translation2d getClosestBall() {
        return latestBallPositions.stream()
            .min((a, b) -> Double.compare(a.getNorm(), b.getNorm()))
            .orElse(null);
    }

    /**
     * Gets the horizontal angle to the closest detected ball.
     *
     * @return Angle in degrees (positive = left, negative = right), or 0 if none detected
     */
    public double getGamepieceAngle() {
        Translation2d closest = getClosestBall();
        if (closest == null) return 0.0;
        return Math.toDegrees(Math.atan2(closest.getY(), closest.getX()));
    }

    /**
     * Gets the forward distance to the closest detected ball.
     *
     * @return Distance in metres, or 0 if none detected
     */
    public double getGamepieceVerticalAngle() {
        Translation2d closest = getClosestBall();
        if (closest == null) return 0.0;
        return closest.getX();
    }
}
