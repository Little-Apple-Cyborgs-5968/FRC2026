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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
import frc.robot.LimelightHelpers.RawFiducial;

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
 * - Two Limelight 4 cameras for AprilTag detection and MegaTag2 pose estimation
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
    private final StructPublisher<Pose2d> frontPosePublisher;
    private final StructPublisher<Pose2d> backPosePublisher;
    
    // Detected AprilTag positions (for AdvantageScope)
    private final StructArrayPublisher<Pose3d> frontTagsPublisher;
    private final StructArrayPublisher<Pose3d> backTagsPublisher;
    
    // ==================== LATEST ESTIMATES ====================
    private PoseEstimate latestFrontEstimate = null;
    private PoseEstimate latestBackEstimate = null;

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
        var visionTable = ntInstance.getTable("Vision");
        
        frontPosePublisher = visionTable.getStructTopic("FrontEstimatedPose", Pose2d.struct).publish();
        backPosePublisher = visionTable.getStructTopic("BackEstimatedPose", Pose2d.struct).publish();
        
        frontTagsPublisher = visionTable.getStructArrayTopic("FrontDetectedTags", Pose3d.struct).publish();
        backTagsPublisher = visionTable.getStructArrayTopic("BackDetectedTags", Pose3d.struct).publish();

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
        LimelightHelpers.setPipelineIndex(Constants.Vision.kLimelightFrontName, 0);
        LimelightHelpers.setPipelineIndex(Constants.Vision.kLimelightBackName, 0);
        
        // Set gamepiece Limelight to neural network pipeline
        LimelightHelpers.setPipelineIndex(Constants.Vision.kLimelightGamepieceName, 0);
        
        // Turn off LEDs for AprilTag cameras (they don't need them)
        LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.kLimelightFrontName);
        LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.kLimelightBackName);
        
        // Configure camera positions relative to robot center using Transform3d constants
        // These override the web GUI values - comment out to use GUI values instead
        if (Constants.Vision.kSetCameraPosesFromCode) {
            setCameraPoseFromTransform(Constants.Vision.kLimelightFrontName, Constants.Vision.kLimelightFrontPosition);
            setCameraPoseFromTransform(Constants.Vision.kLimelightBackName, Constants.Vision.kLimelightBackPosition);
            setCameraPoseFromTransform(Constants.Vision.kLimelightGamepieceName, Constants.Vision.kLimelightGamepiecePosition);
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
        processAprilTagVision(Constants.Vision.kLimelightFrontName, true);
        processAprilTagVision(Constants.Vision.kLimelightBackName, false);
        
        // Log detected AprilTags to NetworkTables
        logDetectedTags();
        
        // ==================== GAMEPIECE DETECTION ====================
        // TODO: Implement gamepiece detection using Limelight 2 with Google Coral
        // 
        // The gamepiece detection should:
        // 1. Use LimelightHelpers.getRawDetections(Constants.Vision.kLimelightGamepieceName)
        //    to get neural network detections
        // 2. Filter detections by confidence and class ID
        // 3. Calculate 3D position of gamepieces relative to robot
        // 4. Publish detected gamepiece positions to NetworkTables for logging
        // 5. Provide methods for commands to query:
        //    - hasGamepiece() - whether a gamepiece is detected
        //    - getClosestGamepiece() - Translation2d to nearest gamepiece
        //    - getGamepieceAngle() - angle to gamepiece for auto-alignment
        //
        // Example structure:
        // processGamepieceDetection();
        // logGamepieceDetections();
    }

    /**
     * Updates the robot orientation on all AprilTag Limelights for MegaTag2.
     * This is required for accurate pose estimation with MegaTag2.
     */
    private void updateRobotOrientation() {
        Rotation2d heading = headingSupplier.get();
        double yawDegrees = heading.getDegrees();
        
        // Set robot orientation for MegaTag2 on both AprilTag cameras
        LimelightHelpers.SetRobotOrientation(
            Constants.Vision.kLimelightFrontName,
            yawDegrees, 0, 0, 0, 0, 0
        );
        LimelightHelpers.SetRobotOrientation(
            Constants.Vision.kLimelightBackName,
            yawDegrees, 0, 0, 0, 0, 0
        );
    }

    /**
     * Processes AprilTag vision data from a single Limelight and feeds it to the pose estimator.
     * 
     * @param limelightName The NetworkTables name of the Limelight
     * @param isFront Whether this is the front camera (for logging purposes)
     */
    private void processAprilTagVision(String limelightName, boolean isFront) {
        // Get pose estimate based on configuration
        PoseEstimate estimate;
        if (Constants.Vision.kUseMegaTag2) {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        } else {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        }

        // Store for logging
        if (isFront) {
            latestFrontEstimate = estimate;
        } else {
            latestBackEstimate = estimate;
        }

        // Validate the estimate
        if (!isValidEstimate(estimate)) {
            return;
        }

        // Log the estimated pose to NetworkTables
        if (isFront) {
            frontPosePublisher.set(estimate.pose);
        } else {
            backPosePublisher.set(estimate.pose);
        }

        // Calculate standard deviations based on number of tags
        Matrix<N3, N1> stdDevs = calculateStdDevs(estimate);

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
     * @return Standard deviation matrix for the pose estimator
     */
    private Matrix<N3, N1> calculateStdDevs(PoseEstimate estimate) {
        if (estimate.tagCount >= 2) {
            // Multi-tag: more accurate
            return Constants.Vision.kMultiTagStdDevs;
        } else {
            // Single tag: less accurate, scale by distance
            double distanceScale = Math.max(1.0, estimate.avgTagDist / 2.0);
            return Constants.Vision.kSingleTagStdDevs.times(distanceScale);
        }
    }

    /**
     * Logs detected AprilTag positions to NetworkTables for AdvantageScope visualization.
     */
    private void logDetectedTags() {
        // Log front camera tags
        List<Pose3d> frontTags = getDetectedTagPoses(latestFrontEstimate);
        frontTagsPublisher.set(frontTags.toArray(new Pose3d[0]));

        // Log back camera tags
        List<Pose3d> backTags = getDetectedTagPoses(latestBackEstimate);
        backTagsPublisher.set(backTags.toArray(new Pose3d[0]));
    }

    /**
     * Extracts the 3D poses of detected AprilTags from a pose estimate.
     * 
     * @param estimate The pose estimate containing raw fiducial data
     * @return List of Pose3d objects representing detected tag positions
     */
    private List<Pose3d> getDetectedTagPoses(PoseEstimate estimate) {
        List<Pose3d> tagPoses = new ArrayList<>();
        
        if (estimate == null || estimate.rawFiducials == null) {
            return tagPoses;
        }

        for (RawFiducial fiducial : estimate.rawFiducials) {
            // Get the field pose of this AprilTag from the loaded field map
            Pose3d tagPose = getAprilTagFieldPose(fiducial.id);
            if (tagPose != null) {
                tagPoses.add(tagPose);
            }
        }

        return tagPoses;
    }

    /**
     * Gets the field pose of an AprilTag by its ID from the loaded .fmap file.
     * 
     * @param tagId The AprilTag ID
     * @return The 3D field pose of the tag, or null if not found
     */
    private Pose3d getAprilTagFieldPose(int tagId) {
        return aprilTagFieldMap.get(tagId);
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
     * Gets the latest pose estimate from the front camera.
     * @return The latest PoseEstimate, or null if none available
     */
    public PoseEstimate getLatestFrontEstimate() {
        return latestFrontEstimate;
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
        boolean frontHasTarget = latestFrontEstimate != null && latestFrontEstimate.tagCount > 0;
        boolean backHasTarget = latestBackEstimate != null && latestBackEstimate.tagCount > 0;
        return frontHasTarget || backHasTarget;
    }

    /**
     * Gets the total number of AprilTags currently visible across all cameras.
     * @return The total tag count
     */
    public int getTotalTagCount() {
        int count = 0;
        if (latestFrontEstimate != null) count += latestFrontEstimate.tagCount;
        if (latestBackEstimate != null) count += latestBackEstimate.tagCount;
        return count;
    }

    // ==================== GAMEPIECE DETECTION PLACEHOLDER METHODS ====================
    // TODO: Implement these methods when adding gamepiece detection
    
    /**
     * Checks if a gamepiece is currently detected.
     * @return true if a gamepiece is visible
     */
    public boolean hasGamepieceTarget() {
        // TODO: Implement gamepiece detection
        // Use LimelightHelpers.getRawDetections(Constants.Vision.kLimelightGamepieceName)
        // Filter by confidence threshold and class ID
        return false;
    }

    /**
     * Gets the horizontal angle to the closest detected gamepiece.
     * @return Angle in degrees (positive = left, negative = right), or 0 if none detected
     */
    public double getGamepieceAngle() {
        // TODO: Implement gamepiece detection
        // Return LimelightHelpers.getTX(Constants.Vision.kLimelightGamepieceName) if valid
        return 0.0;
    }

    /**
     * Gets the vertical angle to the closest detected gamepiece.
     * @return Angle in degrees (positive = up, negative = down), or 0 if none detected
     */
    public double getGamepieceVerticalAngle() {
        // TODO: Implement gamepiece detection
        // Return LimelightHelpers.getTY(Constants.Vision.kLimelightGamepieceName) if valid
        return 0.0;
    }
}
