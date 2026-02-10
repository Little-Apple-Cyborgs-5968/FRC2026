package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * SYOMDrive (Synchronized Yaw-Optimized Motion Drive) Command
 * 
 * This command automatically rotates the robot to face the direction it's traveling,
 * keeping the intake (front of robot) pointed forward during movement.
 * This is ideal for game piece collection while driving.
 * 
 * Features:
 * - Field-centric driving with automatic heading alignment
 * - Smooth proportional rotation to face travel direction
 * - Minimum velocity threshold to prevent jitter when stationary
 * - Seamless integration with existing drivetrain
 */
public class SYOMDriveCommand extends Command {
    
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier velocityXSupplier;
    private final DoubleSupplier velocityYSupplier;
    private final SwerveRequest.FieldCentric driveRequest;
    
    // Rotation control parameters
    private final double kRotationKp = Constants.Swerve.kSYOMDriveRotationKp;
    private final double kMinVelocity = Constants.Swerve.kSYOMDriveMinVelocity;
    
    /**
     * Creates a new SYOMDrive command
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @param velocityXSupplier Supplier for X velocity (m/s, field-relative)
     * @param velocityYSupplier Supplier for Y velocity (m/s, field-relative)
     * @param maxSpeed Maximum speed for deadband calculation (m/s)
     * @param maxAngularRate Maximum angular rate for deadband calculation (rad/s)
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
        
        // Create the underlying field-centric drive request
        this.driveRequest = new SwerveRequest.FieldCentric()
                .withDeadband(maxSpeed * 0.1)
                .withRotationalDeadband(maxAngularRate * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        addRequirements(drivetrain);
    }
    
    @Override
    public void execute() {
        // Get requested velocities
        double vx = velocityXSupplier.getAsDouble();
        double vy = velocityYSupplier.getAsDouble();
        
        // Calculate velocity magnitude
        double velocityMagnitude = Math.sqrt(vx * vx + vy * vy);
        
        // Calculate rotational rate
        double rotationalRate = 0;
        
        if (velocityMagnitude > kMinVelocity) {
            // Calculate the direction of travel (desired heading)
            Rotation2d desiredHeading = new Rotation2d(vx, vy);
            
            // Get current heading
            Rotation2d currentHeading = drivetrain.getState().Pose.getRotation();
            
            // Calculate heading error
            double headingError = desiredHeading.minus(currentHeading).getRadians();
            
            // Normalize to [-π, π]
            while (headingError > Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;
            
            // Apply proportional control for smooth rotation
            rotationalRate = headingError * kRotationKp;
        }
        // If below minimum velocity, rotationalRate stays 0 (no rotation)
        
        // Apply the drive request with calculated rotation
        drivetrain.setControl(
            driveRequest
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(rotationalRate)
        );
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when command ends
        drivetrain.setControl(new SwerveRequest.Idle());
    }
    
    @Override
    public boolean isFinished() {
        return false; // This command runs until interrupted
    }
}

