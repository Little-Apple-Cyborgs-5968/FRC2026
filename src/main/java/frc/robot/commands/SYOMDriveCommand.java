package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * SYOMDrive (Synchronized Yaw-Optimized Motion Drive) Command
 *
 * Automatically rotates the robot to face the direction it is traveling,
 * keeping the intake (front of robot) pointed forward during movement.
 *
 * Features:
 * - Field-centric driving with automatic heading alignment
 * - Speed-gated rotation: auto-turn only activates above kSYOMDriveMinVelocity
 * - Slew-rate-limited rotation output for smooth acceleration / deceleration
 * - Proportional heading correction
 * - Seamless integration with existing CTRE drivetrain
 */
public class SYOMDriveCommand extends Command {

    // ── Subsystem / IO ────────────────────────────────────────────────────────
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier velocityXSupplier;
    private final DoubleSupplier velocityYSupplier;
    private final SwerveRequest.FieldCentric driveRequest;

    // ── Constants (pulled from Constants.Swerve) ──────────────────────────────
    private final double kRotationKp  = Constants.Swerve.kSYOMDriveRotationKp;
    private final double kMinVelocity = Constants.Swerve.kSYOMDriveMinVelocity;
    private final double kMaxRotVel   = Constants.Swerve.kSYOMDriveMaxRotationalVelocity;

    // ── Slew-rate limiter for smooth rotation output ──────────────────────────
    // Limits how fast the rotational rate output can change (rad/s per second)
    private final SlewRateLimiter rotSlewLimiter =
            new SlewRateLimiter(Constants.Swerve.kSYOMDriveSlewRateRadPerSec);

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * Creates a new SYOMDrive command.
     *
     * @param drivetrain        The swerve drivetrain subsystem
     * @param velocityXSupplier Supplier for X velocity (m/s, field-relative)
     * @param velocityYSupplier Supplier for Y velocity (m/s, field-relative)
     * @param maxSpeed          Maximum speed used for deadband calculation (m/s)
     * @param maxAngularRate    Maximum angular rate used for deadband (rad/s)
     */
    public SYOMDriveCommand(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier velocityXSupplier,
            DoubleSupplier velocityYSupplier,
            double maxSpeed,
            double maxAngularRate) {

        this.drivetrain = drivetrain;
        this.velocityXSupplier = velocityXSupplier;
        this.velocityYSupplier = velocityYSupplier;

        this.driveRequest = new SwerveRequest.FieldCentric()
                .withDeadband(maxSpeed * 0.1)
                .withRotationalDeadband(maxAngularRate * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain);
    }

    // ── Command lifecycle ─────────────────────────────────────────────────────

    @Override
    public void initialize() {
        // Reset slew limiter so there is no velocity carry-over from a previous run
        rotSlewLimiter.reset(0);
    }

    @Override
    public void execute() {
        double vx = velocityXSupplier.getAsDouble();
        double vy = velocityYSupplier.getAsDouble();

        double velocityMagnitude = Math.sqrt(vx * vx + vy * vy);

        double targetRotRate;

        if (velocityMagnitude > kMinVelocity) {
            // ── Robot is moving fast enough: engage auto-rotation ──────────────

            // Desired heading = direction of the travel vector (operator-perspective)
            Rotation2d desiredHeading = new Rotation2d(vx, vy);

            // Adjust desired heading if on the Red Alliance
            var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
                desiredHeading = desiredHeading.rotateBy(Rotation2d.fromDegrees(180));
            }

            // Current robot heading from odometry
            Rotation2d currentHeading = drivetrain.getState().Pose.getRotation();

            // Shortest-path heading error, normalised to (-π, π]
            double headingError = MathUtil.angleModulus(
                    desiredHeading.minus(currentHeading).getRadians());

            // P-controller: proportional output clamped to max rotational velocity
            targetRotRate = MathUtil.clamp(
                    headingError * kRotationKp,
                    -kMaxRotVel, kMaxRotVel);

        } else {
            // ── Below speed threshold: command zero rotation ──────────────────
            targetRotRate = 0;
        }

        // Slew-rate limiter smooths the transition to avoid jerky rotation
        double rotationalRate = rotSlewLimiter.calculate(targetRotRate);

        drivetrain.setControl(
                driveRequest
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withRotationalRate(rotationalRate));
    }

    @Override
    public void end(boolean interrupted) {
        rotSlewLimiter.reset(0);
        drivetrain.setControl(new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}

