// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
    import edu.wpi.first.util.datalog.DataLogWriter;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Handles all SmartDashboard/Shuffleboard visualizations.
 * Call update() periodically to refresh the robot pose.
 */
public class DashboardPublisher {
    private final Field2d m_field = new Field2d();
    private final Field2d m_autoPreviewField = new Field2d();
    private final CommandSwerveDrivetrain m_drivetrain;
    private final SendableChooser<Command> m_autoChooser;
    private String m_lastAutoName = "";

    public DashboardPublisher(CommandSwerveDrivetrain drivetrain, SendableChooser<Command> autoChooser) {
        m_drivetrain = drivetrain;
        m_autoChooser = autoChooser;

        //starts data logger (writes downs to a WpiLog file )
        configureWpiLogging();
        
        SmartDashboard.putData("Robot Field", m_field);
        SmartDashboard.putData("Auto Preview", m_autoPreviewField);
        
        // Subscribe to PathPlanner's active path
        configurePathPlannerLogging();

        // Initialize swerve drive widget
        initSwerveDriveWidget();
    }

    private void configureWpiLogging(){
        // Start data logger - save to USB drive if available, otherwise use default location
        if (new java.io.File("/u/").exists()) {
            DataLogManager.start("/u/logs");
        } else if (new java.io.File("/media/sda1/").exists()) {
            DataLogManager.start("/media/sda1/logs");
        } else {
            DataLogManager.start(); // Default location
        }

        // Log DriverStation data (joystick inputs, match time, etc.)
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    /** Sets up PathPlanner to automatically log paths to our Field2d */
    private void configurePathPlannerLogging() {
        // PathPlanner will call these whenever a path starts/ends
        com.pathplanner.lib.util.PathPlannerLogging.setLogActivePathCallback((poses) -> {
            m_field.getObject("activePath").setPoses(poses);
        });

        com.pathplanner.lib.util.PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Intentionally empty - using drivetrain odometry
        });
    }

    /** Call this every robot periodic cycle */
    public void update() {
        m_field.setRobotPose(m_drivetrain.getState().Pose);
        
        // Update auto preview if selection changed
        updateAutoPreview();
    }

    /** Updates the auto preview field when a new auto is selected */
    private void updateAutoPreview() {
        String currentAutoName = m_autoChooser.getSelected() != null 
            ? m_autoChooser.getSelected().getName() 
            : "";
        
        // Only update if the selection changed
        if (!currentAutoName.equals(m_lastAutoName)) {
            m_lastAutoName = currentAutoName;
            displayAutoPath(currentAutoName);
        }
    }

    /** Displays all paths for a given auto name */
    private void displayAutoPath(String autoName) {
        // Clear previous paths
        for (int i = 0; i < 10; i++) {
            m_autoPreviewField.getObject("path" + i).setPoses(List.of());
        }
        
        if (autoName.isEmpty()) {
            return;
        }

        try {
            // Get all path names from the auto
            List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
            
            // Display each path with a different object name
            for (int i = 0; i < paths.size(); i++) {
                PathPlannerPath path = paths.get(i);
                List<Pose2d> poses = path.getPathPoses();
                m_autoPreviewField.getObject("path" + i).setPoses(poses);
            }
            
            // Show starting pose as robot position
            if (!paths.isEmpty() && !paths.get(0).getPathPoses().isEmpty()) {
                m_autoPreviewField.setRobotPose(paths.get(0).getPathPoses().get(0));
            }
            
        } catch (Exception e) {
            System.out.println("Could not load auto preview for: " + autoName);
        }
    }

    /** Display a WPILib trajectory on the field */
    public void showTrajectory(edu.wpi.first.math.trajectory.Trajectory trajectory) {
        m_field.getObject("trajectory").setTrajectory(trajectory);
    }

    /** Display a list of poses as a path */
    public void showPath(String name, List<Pose2d> poses) {
        m_field.getObject(name).setPoses(poses);
    }

    /** Clear a displayed path */
    public void clearPath(String name) {
        m_field.getObject(name).setPoses(List.of());
    }

    /** Initializes the swerve drive widget for Elastic dashboard */
    public void initSwerveDriveWidget() {
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", 
                    () -> flipAngleIfRed(m_drivetrain.getState().ModuleStates[0].angle.getRadians()), null);
                builder.addDoubleProperty("Front Left Velocity", 
                    () -> flipVelocityIfRed(m_drivetrain.getState().ModuleStates[0].speedMetersPerSecond), null);

                builder.addDoubleProperty("Front Right Angle", 
                    () -> flipAngleIfRed(m_drivetrain.getState().ModuleStates[1].angle.getRadians()), null);
                builder.addDoubleProperty("Front Right Velocity", 
                    () -> flipVelocityIfRed(m_drivetrain.getState().ModuleStates[1].speedMetersPerSecond), null);

                builder.addDoubleProperty("Back Left Angle", 
                    () -> flipAngleIfRed(m_drivetrain.getState().ModuleStates[2].angle.getRadians()), null);
                builder.addDoubleProperty("Back Left Velocity", 
                    () -> flipVelocityIfRed(m_drivetrain.getState().ModuleStates[2].speedMetersPerSecond), null);

                builder.addDoubleProperty("Back Right Angle", 
                    () -> flipAngleIfRed(m_drivetrain.getState().ModuleStates[3].angle.getRadians()), null);
                builder.addDoubleProperty("Back Right Velocity", 
                    () -> flipVelocityIfRed(m_drivetrain.getState().ModuleStates[3].speedMetersPerSecond), null);

                builder.addDoubleProperty("Robot Angle", 
                    () -> flipAngleIfRed(m_drivetrain.getState().Pose.getRotation().getRadians()), null);
            }
        });
    }

    /** Flips angle by 180 degrees if on red alliance */
    private double flipAngleIfRed(double angleRadians) {
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            return angleRadians + Math.PI;
        }
        return angleRadians;
    }

    /** Negates velocity if on red alliance (for flipped display) */
    private double flipVelocityIfRed(double velocity) {
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            return -velocity;
        }
        return velocity;
    }
}
