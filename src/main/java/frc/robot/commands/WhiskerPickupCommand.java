// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

import java.util.List;

/**
 * WhiskerPickupCommand — autonomous Fuel-ball collection using the "whisker" scoring algorithm.
 *
 * <h2>Algorithm overview</h2>
 * <ol>
 *   <li>Each loop, read all currently-detected ball positions (robot-relative) from Vision.</li>
 *   <li>Fan a set of virtual "whiskers" out from the robot in discrete angular steps across a
 *       forward-facing arc (±{@code kWhiskerFanHalfAngleDeg} from robot forward).</li>
 *   <li>Each whisker is a straight-line corridor of half-width {@code intakeWidth/2} and
 *       length {@code kWhiskerLengthMeters}.</li>
 *   <li>Score each whisker by the number of balls whose centres fall inside its corridor,
 *       weighted by {@code kWhiskerBallValue}.</li>
 *   <li>Drive forward while steering toward the highest-scoring whisker angle with a
 *       P-controller on heading error.</li>
 *   <li>If no whisker exceeds {@code kPickupMinScoreThreshold} the robot holds position
 *       (i.e. waits for new detections rather than driving blindly).</li>
 * </ol>
 *
 * <h2>Toggle usage</h2>
 * Bind {@code new WhiskerPickupCommand(drivetrain, vision)} to a button with
 * {@code .toggleOnTrue()} in RobotContainer.
 */
public class WhiskerPickupCommand extends Command {

    // ==================== DEPENDENCIES ====================
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;

    // ==================== DRIVE REQUEST ====================
    // Robot-centric so the robot always drives relative to its own forward direction
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // ==================== STATE ====================
    /** The robot-relative angle (radians) of the best whisker from the most-recent loop. */
    private double bestWhiskerAngleRad = 0.0;

    /** Score of the best whisker from the most-recent loop. */
    private double bestWhiskerScore = 0.0;

    // ==================== TELEMETRY ====================
    private final DoublePublisher bestAnglePublisher;
    private final DoublePublisher bestScorePublisher;

    /**
     * Creates a WhiskerPickupCommand.
     *
     * @param drivetrain The swerve drivetrain (required subsystem)
     * @param vision     The Vision subsystem providing ball positions
     */
    public WhiskerPickupCommand(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision     = vision;
        addRequirements(drivetrain);

        var table = NetworkTableInstance.getDefault().getTable("VISION/Whisker");
        bestAnglePublisher = table.getDoubleTopic("BestAngleDeg").publish();
        bestScorePublisher = table.getDoubleTopic("BestScore").publish();
    }

    // ==================== COMMAND LIFECYCLE ====================

    @Override
    public void initialize() {
        bestWhiskerAngleRad = 0.0;
        bestWhiskerScore    = 0.0;
    }

    @Override
    public void execute() {
        List<Translation2d> balls = vision.getBallPositions();

        // Evaluate all whiskers and find the best one
        evaluateWhiskers(balls);

        // Publish telemetry
        bestAnglePublisher.set(Math.toDegrees(bestWhiskerAngleRad));
        bestScorePublisher.set(bestWhiskerScore);

        if (bestWhiskerScore < Constants.Vision.kPickupMinScoreThreshold) {
            // No meaningful target — stop driving, wait for detections
            drivetrain.setControl(driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
            return;
        }

        // The best whisker angle is the desired robot heading offset from current forward.
        // Apply a P-controller to rotate toward it while driving forward.
        double rotationalRate = MathUtil.clamp(
            bestWhiskerAngleRad * Constants.Vision.kPickupRotationKp,
            -Constants.Vision.kPickupMaxRotationalRateRadS,
             Constants.Vision.kPickupMaxRotationalRateRadS
        );

        drivetrain.setControl(driveRequest
            .withVelocityX(Constants.Vision.kPickupDriveSpeed)  // always drive forward (robot-centric +X)
            .withVelocityY(0)
            .withRotationalRate(rotationalRate));
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the mode is toggled off or interrupted
        drivetrain.setControl(driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        // Runs until toggled off — never self-terminates
        return false;
    }

    // ==================== WHISKER ALGORITHM ====================

    /**
     * Fans whiskers across the forward arc and records the best angle and score.
     *
     * <p>Each whisker is defined by:
     * <ul>
     *   <li>Direction: an angle {@code theta} from robot forward (robot +X axis), swept in
     *       discrete steps of {@code kWhiskerAngleStepDeg} across
     *       [{@code -kWhiskerFanHalfAngleDeg}, {@code +kWhiskerFanHalfAngleDeg}].</li>
     *   <li>Corridor half-width: {@code kIntakeWidthMeters / 2}.</li>
     *   <li>Length: {@code kWhiskerLengthMeters}.</li>
     * </ul>
     *
     * <p>A ball is "inside" a whisker if:
     * <ol>
     *   <li>Its projection onto the whisker direction is in [0, kWhiskerLengthMeters].</li>
     *   <li>Its perpendicular distance from the whisker centreline is ≤ half-width.</li>
     * </ol>
     *
     * <p>Convention: robot-relative +X = forward, +Y = left. Positive whisker angles sweep
     * toward the left of the robot (counter-clockwise from robot forward).
     *
     * @param balls Current robot-relative ball positions from Vision
     */
    private void evaluateWhiskers(List<Translation2d> balls) {
        double halfFanRad   = Math.toRadians(Constants.Vision.kWhiskerFanHalfAngleDeg);
        double stepRad      = Math.toRadians(Constants.Vision.kWhiskerAngleStepDeg);
        double halfWidth    = Constants.Vision.kIntakeWidthMeters / 2.0;
        double whiskerLen   = Constants.Vision.kWhiskerLengthMeters;
        double ballValue    = Constants.Vision.kWhiskerBallValue;

        double topScore     = -1.0;
        double topAngleRad  = 0.0;

        // Sweep from -halfFan to +halfFan
        for (double theta = -halfFanRad; theta <= halfFanRad + 1e-9; theta += stepRad) {

            // Unit vector along this whisker in robot frame
            double wx = Math.cos(theta); // forward component
            double wy = Math.sin(theta); // lateral component (positive = left)

            double score = 0.0;

            for (Translation2d ball : balls) {
                double bx = ball.getX(); // robot-relative forward
                double by = ball.getY(); // robot-relative left

                // Project ball onto whisker direction
                double along = bx * wx + by * wy;

                // Check ball is within the whisker's length range
                if (along < 0 || along > whiskerLen) continue;

                // Perpendicular distance from the whisker centreline
                double perp = Math.abs(bx * (-wy) + by * wx);

                if (perp <= halfWidth) {
                    score += ballValue;
                }
            }

            if (score > topScore) {
                topScore    = score;
                topAngleRad = theta;
            }
        }

        bestWhiskerScore    = topScore;
        bestWhiskerAngleRad = topAngleRad;
    }
}
