package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * SetYawCommand — Snaps the robot to a field-relative target heading while
 * allowing full driver-controlled translation.
 *
 * <p>Uses CTRE's built-in {@link SwerveRequest.FieldCentricFacingAngle} request,
 * which internally runs a PhoenixPIDController on the robot heading. Steer modules
 * use MotionMagicExpo for smooth, jerk-limited azimuth control, and drive motors
 * use closed-loop Velocity control.
 *
 * <p>All tunable values live in {@link frc.robot.Constants.Swerve}.
 *
 * <p>The command finishes when the heading error is within
 * {@link frc.robot.Constants.Swerve#kHeadingToleranceDegrees} for one loop cycle.
 * If you want it to hold indefinitely (e.g. as a whileTrue binding), simply pass
 * {@code finishOnTarget = false}.
 *
 * <h3>Typical usage</h3>
 * <pre>{@code
 * // Snap to 90° and hold while the button is held
 * joystick.b().whileTrue(new SetYawCommand(
 *     drivetrain,
 *     () -> -joystick.getLeftY() * MaxSpeed,
 *     () -> -joystick.getLeftX() * MaxSpeed,
 *     MaxSpeed,
 *     90.0,
 *     false   // don't finish automatically — keep holding while button pressed
 * ));
 *
 * // Snap to 0° and finish once aligned
 * new SetYawCommand(drivetrain, vxSupplier, vySupplier, maxSpeed, 0.0, true);
 * }</pre>
 */
public class SetYawCommand extends Command {

    // ── Subsystem ──────────────────────────────────────────────────────────────
    private final CommandSwerveDrivetrain drivetrain;

    // ── Translation suppliers ──────────────────────────────────────────────────
    private final DoubleSupplier velocityXSupplier; // Field-relative X velocity (m/s)
    private final DoubleSupplier velocityYSupplier; // Field-relative Y velocity (m/s)

    // ── Target heading ─────────────────────────────────────────────────────────
    // Supplier so dynamic targets (e.g. nearest-45°) are resolved at
    // initialize() time, not at construction time.
    private final Supplier<Rotation2d> targetHeadingSupplier;
    private Rotation2d targetHeading; // resolved in initialize()

    // ── Finish-on-target flag ──────────────────────────────────────────────────
    private final boolean finishOnTarget;

    // ── Heading tolerance (converted once at construction) ─────────────────────
    private final double toleranceRadians =
            Units.degreesToRadians(Constants.Swerve.kHeadingToleranceDegrees);

    // ── CTRE swerve request ────────────────────────────────────────────────────
    // FieldCentricFacingAngle lets the driver translate freely while CTRE's
    // internal PhoenixPIDController manages the yaw target autonomously.
    private final SwerveRequest.FieldCentricFacingAngle headingRequest;

    // ── WPILib PID controller (drives isFinished check) ───────────────────────
    // We keep a lightweight WPILib PIDController solely to query atSetpoint().
    // The actual heading control is performed by the CTRE request.
    private final PIDController toleranceChecker;

    // ──────────────────────────────────────────────────────────────────────────
    // Primary constructor (fixed target angle)
    // ──────────────────────────────────────────────────────────────────────────

    /**
     * @param drivetrain        The swerve drivetrain subsystem.
     * @param velocityXSupplier Field-relative X velocity in m/s (e.g. left stick Y axis,
     *                          already deadbanded and scaled to MaxSpeed).
     * @param velocityYSupplier Field-relative Y velocity in m/s (e.g. left stick X axis,
     *                          already deadbanded and scaled to MaxSpeed).
     * @param maxSpeed          Robot's maximum speed in m/s — used to scale the
     *                          translation deadband inside the CTRE request.
     * @param targetDegrees     Desired robot heading in degrees, field-relative.
     *                          0° = away from driver wall (WPILib convention).
     * @param finishOnTarget    If {@code true} the command ends once the heading
     *                          is within tolerance.  If {@code false} it runs until
     *                          interrupted (good for whileTrue bindings).
     */
    public SetYawCommand(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier velocityXSupplier,
            DoubleSupplier velocityYSupplier,
            double maxSpeed,
            double targetDegrees,
            boolean finishOnTarget) {
        this(drivetrain, velocityXSupplier, velocityYSupplier, maxSpeed,
             () -> Rotation2d.fromDegrees(targetDegrees), finishOnTarget);
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Internal constructor (dynamic target via Supplier — resolved at init)
    // ──────────────────────────────────────────────────────────────────────────
    private SetYawCommand(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier velocityXSupplier,
            DoubleSupplier velocityYSupplier,
            double maxSpeed,
            Supplier<Rotation2d> targetHeadingSupplier,
            boolean finishOnTarget) {

        this.drivetrain            = drivetrain;
        this.velocityXSupplier     = velocityXSupplier;
        this.velocityYSupplier     = velocityYSupplier;
        this.targetHeadingSupplier = targetHeadingSupplier;
        this.finishOnTarget        = finishOnTarget;

        // Build the CTRE facing-angle request once (PID object allocation stays
        // off the hot path).
        this.headingRequest = new SwerveRequest.FieldCentricFacingAngle()
                // Translation deadband — matches the 10 % used by the default drive command
                .withDeadband(maxSpeed * Constants.Swerve.kFacingAngleTranslationDeadband)
                // Velocity closed-loop for smooth, accurate drive speed
                .withDriveRequestType(DriveRequestType.Velocity)
                // MotionMagicExpo for jerk-limited, snappy steer response
                .withSteerRequestType(SteerRequestType.MotionMagicExpo);

        // Configure the CTRE heading PID with values from Constants
        headingRequest.HeadingController.setPID(
                Constants.Swerve.kHeadingKP,
                Constants.Swerve.kHeadingKI,
                Constants.Swerve.kHeadingKD);
        headingRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        // Tolerance checker — mirrors the CTRE PID config for atSetpoint() checks
        toleranceChecker = new PIDController(
                Constants.Swerve.kHeadingKP,
                Constants.Swerve.kHeadingKI,
                Constants.Swerve.kHeadingKD);
        toleranceChecker.enableContinuousInput(-Math.PI, Math.PI);
        toleranceChecker.setTolerance(toleranceRadians);

        addRequirements(drivetrain);
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Static factory helpers
    // ──────────────────────────────────────────────────────────────────────────

    /**
     * Holds the given heading until interrupted. Ideal for {@code whileTrue} bindings.
     */
    public static SetYawCommand hold(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier velocityXSupplier,
            DoubleSupplier velocityYSupplier,
            double maxSpeed,
            double targetDegrees) {
        return new SetYawCommand(
                drivetrain, velocityXSupplier, velocityYSupplier,
                maxSpeed, targetDegrees, false);
    }

    /**
     * Snaps to the given heading and finishes once within tolerance.
     * Ideal for sequential command compositions and autonomous routines.
     */
    public static SetYawCommand snap(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier velocityXSupplier,
            DoubleSupplier velocityYSupplier,
            double maxSpeed,
            double targetDegrees) {
        return new SetYawCommand(
                drivetrain, velocityXSupplier, velocityYSupplier,
                maxSpeed, targetDegrees, true);
    }

    /**
     * Snaps the robot to whichever multiple of 45° is closest to its heading
     * <em>at the moment the button is pressed</em> (evaluated in
     * {@link #initialize()}, not at construction time), then holds there until
     * the button is released.
     *
     * <p>Candidate lock angles: 0°, ±45°, ±90°, ±135°, 180°
     * (WPILib field-relative — 0° = away from driver wall).
     *
     * @param drivetrain        The swerve drivetrain subsystem.
     * @param velocityXSupplier Field-relative X velocity in m/s, already deadbanded.
     * @param velocityYSupplier Field-relative Y velocity in m/s, already deadbanded.
     * @param maxSpeed          Robot's maximum speed in m/s.
     */
    public static SetYawCommand snapToNearest45(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier velocityXSupplier,
            DoubleSupplier velocityYSupplier,
            double maxSpeed) {
        return new SetYawCommand(
                drivetrain, velocityXSupplier, velocityYSupplier,
                maxSpeed,
                // Supplier is called in initialize() — reads heading when button is pressed
                () -> {
                    double currentDeg = drivetrain.getState().Pose.getRotation().getDegrees();
                    double nearest45  = Math.round(currentDeg / 45.0) * 45.0;
                    return Rotation2d.fromDegrees(nearest45);
                },
                false); // hold until interrupted
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Command lifecycle
    // ──────────────────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        // Resolve the target heading now (correct for dynamic suppliers like nearest-45)
        targetHeading = targetHeadingSupplier.get();
        toleranceChecker.reset();
    }

    @Override
    public void execute() {
        drivetrain.setControl(
            headingRequest
                .withVelocityX(velocityXSupplier.getAsDouble())
                .withVelocityY(velocityYSupplier.getAsDouble())
                .withTargetDirection(targetHeading)
        );

        // Keep the tolerance checker current for isFinished()
        toleranceChecker.calculate(
                drivetrain.getState().Pose.getRotation().getRadians(),
                targetHeading.getRadians());
    }

    @Override
    public void end(boolean interrupted) {
        // Hand control back to the default drive command gracefully
        drivetrain.setControl(new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        return finishOnTarget && toleranceChecker.atSetpoint();
    }
}
