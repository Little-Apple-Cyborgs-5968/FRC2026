package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.TurretUtil;

/**
 * DrivetrainShootOnMoveCommand — Rotates the swerve drivetrain to the lead-compensated
 * field heading required to shoot at the hub while the robot is moving, instead of
 * rotating the turret.  The turret is held at 0° (robot-forward) throughout.
 *
 * <p>Mirrors the structure of {@link frc.robot.subsystems.Turret#shootOnMoveCommandTurret}:
 * every loop cycle it calls {@link TurretUtil#computeLeadShotSolution} to get the
 * motion-compensated field angle, then commands the drivetrain to face that angle
 * using CTRE's {@link SwerveRequest.FieldCentricFacingAngle} request (same heading
 * controller already used by {@link SetYawCommand}).
 *
 * <h3>Usage</h3>
 * <pre>{@code
 * // Warm-up phase (trigger held) — flywheel only, drivetrain rotates, turret at 0
 * DrivetrainShootOnMoveCommand cmd = new DrivetrainShootOnMoveCommand(
 *     drivetrain, turret,
 *     () -> -joystick.getLeftY() * MaxSpeed,
 *     () -> -joystick.getLeftX() * MaxSpeed,
 *     MaxSpeed,
 *     () -> drivetrain.getState().Pose,
 *     () -> ChassisSpeeds.fromRobotRelativeSpeeds(
 *             drivetrain.getState().Speeds,
 *             drivetrain.getState().Pose.getRotation()),
 *     TurretUtil.TargetType.HUB
 * );
 * }</pre>
 *
 * <p>Note: this command requires both {@code drivetrain} and {@code turret} so the
 * scheduler will not allow it to run concurrently with any other command that uses
 * either subsystem (e.g. the default field-centric drive or the turret auto-aim).
 */
public class DrivetrainShootOnMoveCommand extends Command {

    // ── Subsystems ─────────────────────────────────────────────────────────────
    private final CommandSwerveDrivetrain drivetrain;
    private final frc.robot.subsystems.Turret turret;

    // ── Translation suppliers ──────────────────────────────────────────────────
    private final DoubleSupplier velocityXSupplier; // Field-relative X velocity (m/s)
    private final DoubleSupplier velocityYSupplier; // Field-relative Y velocity (m/s)

    // ── Pose / speed suppliers ─────────────────────────────────────────────────
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    // ── Target ─────────────────────────────────────────────────────────────────
    private final TurretUtil.TargetType target;

    // ── CTRE heading request ────────────────────────────────────────────────────
    private final SwerveRequest.FieldCentricFacingAngle headingRequest;

    // ── WPILib PID (used only to expose atSetpoint() for callers) ─────────────
    private final PIDController toleranceChecker;

    // ──────────────────────────────────────────────────────────────────────────
    // Constructor
    // ──────────────────────────────────────────────────────────────────────────

    /**
     * @param drivetrain            The swerve drivetrain subsystem.
     * @param turret                The turret subsystem (will be held at 0° while active).
     * @param velocityXSupplier     Field-relative X translation velocity (m/s), already
     *                              deadbanded and scaled to MaxSpeed.
     * @param velocityYSupplier     Field-relative Y translation velocity (m/s), already
     *                              deadbanded and scaled to MaxSpeed.
     * @param maxSpeed              Robot's maximum speed in m/s — used for the translation
     *                              deadband inside the CTRE request.
     * @param robotPoseSupplier     Supplier for the current robot field pose.
     * @param chassisSpeedsSupplier Supplier for the current field-relative chassis speeds
     *                              (e.g. {@code ChassisSpeeds.fromRobotRelativeSpeeds(...)}).
     * @param target                Which target to aim at.
     */
    public DrivetrainShootOnMoveCommand(
            CommandSwerveDrivetrain drivetrain,
            frc.robot.subsystems.Turret turret,
            DoubleSupplier velocityXSupplier,
            DoubleSupplier velocityYSupplier,
            double maxSpeed,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier,
            TurretUtil.TargetType target) {

        this.drivetrain             = drivetrain;
        this.turret                 = turret;
        this.velocityXSupplier      = velocityXSupplier;
        this.velocityYSupplier      = velocityYSupplier;
        this.robotPoseSupplier      = robotPoseSupplier;
        this.chassisSpeedsSupplier  = chassisSpeedsSupplier;
        this.target                 = target;

        // Build the CTRE facing-angle request (same configuration as SetYawCommand)
        this.headingRequest = new SwerveRequest.FieldCentricFacingAngle()
                .withDeadband(maxSpeed * Constants.Swerve.kFacingAngleTranslationDeadband)
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSteerRequestType(SteerRequestType.MotionMagicExpo);

        headingRequest.HeadingController.setPID(
                Constants.Swerve.kHeadingKP,
                Constants.Swerve.kHeadingKI,
                Constants.Swerve.kHeadingKD);
        headingRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        // Tolerance checker mirrors CTRE PID for atSetpoint() queries
        toleranceChecker = new PIDController(
                Constants.Swerve.kHeadingKP,
                Constants.Swerve.kHeadingKI,
                Constants.Swerve.kHeadingKD);
        toleranceChecker.enableContinuousInput(-Math.PI, Math.PI);
        toleranceChecker.setTolerance(
                Units.degreesToRadians(Constants.Swerve.kHeadingToleranceDegrees));

        addRequirements(drivetrain, turret);
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Command lifecycle
    // ──────────────────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        toleranceChecker.reset();
    }

    @Override
    public void execute() {
        Pose2d robotPose   = robotPoseSupplier.get();
        ChassisSpeeds speeds = chassisSpeedsSupplier.get();

        // Compute lead-compensated shot solution (same algorithm as shootOnMoveCommandTurret)
        TurretUtil.ShotSolution solution = TurretUtil.computeLeadShotSolution(
                robotPose,
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                target);

        // Hold turret at 0° — drivetrain is doing the aiming
        turret.setAngle(0.0);

        if (solution.isValid) {
            // Convert the turret lead angle back to a field-relative heading:
            // The drivetrain must face exactly the direction the turret would have pointed.
            // solution.turretAngleDegrees is the required turret offset from the robot heading,
            // so the robot heading must become: fieldAngleToTarget = robotHeading + turretOffset.
            double requiredRobotHeadingDeg =
                    robotPose.getRotation().getDegrees() + solution.turretAngleDegrees;
            Rotation2d targetHeading = Rotation2d.fromDegrees(requiredRobotHeadingDeg);

            drivetrain.setControl(
                    headingRequest
                            .withVelocityX(velocityXSupplier.getAsDouble())
                            .withVelocityY(velocityYSupplier.getAsDouble())
                            .withTargetDirection(targetHeading));

            // Keep tolerance checker current
            toleranceChecker.calculate(
                    robotPose.getRotation().getRadians(),
                    targetHeading.getRadians());
        } else {
            // Solution invalid (out of range) — still allow driver to translate freely
            // and keep turret at 0; just don't rotate.
            drivetrain.setControl(
                    new SwerveRequest.FieldCentric()
                            .withDeadband(0)
                            .withDriveRequestType(DriveRequestType.Velocity)
                            .withVelocityX(velocityXSupplier.getAsDouble())
                            .withVelocityY(velocityYSupplier.getAsDouble()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Hand control back to the default drive command gracefully
        drivetrain.setControl(new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        // Runs until interrupted (used as a whileTrue binding)
        return false;
    }

    /**
     * Returns {@code true} once the drivetrain heading error is within the configured
     * tolerance.  Callers can gate feeder/spindexer firing on this.
     */
    public boolean isAligned() {
        return toleranceChecker.atSetpoint();
    }
}
