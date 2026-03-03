// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.driverIO;

import frc.robot.utils.GameState;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLogWriter;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
    private final SendableChooser<String> m_pathfindZoneChooser;
    private String m_lastAutoName = "";
    
    // ==================== TUNABLE VALUES ====================
    // Editable on dashboard - no redeploy needed!
    private final NetworkTableEntry tunableKP = 
        NetworkTableInstance.getDefault()
            .getTable("Tuning")
            .getEntry("kP");
    
    private final NetworkTableEntry tunableKI = 
        NetworkTableInstance.getDefault()
            .getTable("Tuning")
            .getEntry("kI");
    
    private final NetworkTableEntry tunableKD = 
        NetworkTableInstance.getDefault()
            .getTable("Tuning")
            .getEntry("kD");
    
    private final NetworkTableEntry tunableKV = 
        NetworkTableInstance.getDefault()
            .getTable("Tuning")
            .getEntry("kV");
    
    private final NetworkTableEntry tunableKA = 
        NetworkTableInstance.getDefault()
            .getTable("Tuning")
            .getEntry("kA");
    
    private final NetworkTableEntry tunableKX = 
        NetworkTableInstance.getDefault()
            .getTable("Tuning")
            .getEntry("kX");
    
    private final NetworkTableEntry tunableSetpoint1 = 
        NetworkTableInstance.getDefault()
            .getTable("Tuning")
            .getEntry("Setpoint1");
    
    private final NetworkTableEntry tunableSetpoint2 = 
        NetworkTableInstance.getDefault()
            .getTable("Tuning")
            .getEntry("Setpoint2");

    public DashboardPublisher(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;

        //creates and puts the auto chooser object from pathplanner autos
        m_autoChooser = AutoBuilder.buildAutoChooser();
        
        //puts auto chooser on smartdashboard for selection
        SmartDashboard.putData("DASHBOARD/Auto Chooser", m_autoChooser);

        // Build the pathfind-zone chooser: Home → trench, Mid → neutral, Opp → opponent zone
        m_pathfindZoneChooser = new SendableChooser<>();
        m_pathfindZoneChooser.setDefaultOption("Home", "Home");
        m_pathfindZoneChooser.addOption("Mid", "Mid");
        m_pathfindZoneChooser.addOption("Opp", "Opp");
        SmartDashboard.putData("DASHBOARD/Pathfind Zone", m_pathfindZoneChooser);

        // Initialize tunable values with defaults
        tunableKP.setDouble(0.0);
        tunableKI.setDouble(0.0);
        tunableKD.setDouble(0.0);
        tunableKV.setDouble(0.0);
        tunableKA.setDouble(0.0);
        tunableKX.setDouble(0.0);
        tunableSetpoint1.setDouble(0.0);
        tunableSetpoint2.setDouble(0.0);

        // Put data in the "DASHBOARD" subfolder on the dashboard (NetworkTables keys support '/')
        SmartDashboard.putData("DASHBOARD/Robot Field", m_field);
        SmartDashboard.putData("DASHBOARD/Auto Preview", m_autoPreviewField);
        SmartDashboard.putData("DASHBOARD/Scheduler", CommandScheduler.getInstance());

        // Initialize the swerve widget in DASHBOARD subfolder
        initSwerveDriveWidget();

        // Subscribe to PathPlanner's active path
        configurePathPlannerLogging();

        //starts data logger (writes downs to a WpiLog file )
        configureWpiLogging();


    }

    /** * Call periodically to update dashboard visualizations */
    public void update() {
    updateRobotField();
    updateAutoPreviewField();
    updateGameState();
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.updateValues();
    }

    /** Gets the current tunable value from the dashboard */
    public double getTunableKP() {
        return tunableKP.getDouble(0.0);
    }

    public double getTunableKI() {
        return tunableKI.getDouble(0.0);
    }

    public double getTunableKD() {
        return tunableKD.getDouble(0.0);
    }

    public double getTunableKV() {
        return tunableKV.getDouble(0.0);
    }

    public double getTunableKA() {
        return tunableKA.getDouble(0.0);
    }

    public double getTunableKX() {
        return tunableKX.getDouble(0.0);
    }

    public double getTunableSetpoint1() {
        return tunableSetpoint1.getDouble(0.0);
    }

    public double getTunableSetpoint2() {
        return tunableSetpoint2.getDouble(0.0);
    }

    public Command getAuto() {
        return m_autoChooser.getSelected();
    }

    /** Returns the currently selected pathfind zone: "Home", "Mid", or "Opp". */
    public String getPathfindZone() {
        String selected = m_pathfindZoneChooser.getSelected();
        return selected != null ? selected : "Home";
    }

//------------------------------------------------------------------------------------
//ROBOT FIELD WIDGET(m_field)
//------------------------------------------------------------------------------------

    /** Updates the robot pose on the Field2d widget */
    private void updateRobotField() {
        m_field.setRobotPose(m_drivetrain.getState().Pose);
    }

    /** Sets up PathPlanner to automatically log paths to our Robot Field2d */
    private void configurePathPlannerLogging() {
        // PathPlanner will call these whenever a path starts/ends
        com.pathplanner.lib.util.PathPlannerLogging.setLogActivePathCallback((poses) -> {
            m_field.getObject("activePath").setPoses(poses);
        });

        com.pathplanner.lib.util.PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Intentionally empty - using drivetrain odometry
        });
    }

//------------------------------------------------------------------------------------
//AUTO CHOOSER AND PREVIEW WIDGET(m_autoChooser,m_autoPreviewField)
//------------------------------------------------------------------------------------

    /** Updates the auto preview field when a new auto is selected */
    private void updateAutoPreviewField() {
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

//------------------------------------------------------------------------------------
//SWERVE WIDGET
//------------------------------------------------------------------------------------
    /** Initializes the swerve drive widget for Elastic dashboard */
    public void initSwerveDriveWidget() {
        SmartDashboard.putData("DASHBOARD/Swerve Drive", new Sendable() {
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
        });
    }

//------------------------------------------------------------------------------------
//GAME STATE,SHIFT, & TIMER DISPLAY
//------------------------------------------------------------------------------------

    /** Publishes the current game state and time left in state to SmartDashboard */
    private void updateGameState() {
        GameState.States currentState = GameState.determineGameState();
        String stateString = (currentState != null) ? currentState.name() : "UNKNOWN";
        SmartDashboard.putString("DASHBOARD/Game State", stateString);
        SmartDashboard.putNumber("DASHBOARD/Time Left In State", GameState.timeRemainingInCurrentState());
    }

//------------------------------------------------------------------------------------
//Data logging to WPI Logs
//------------------------------------------------------------------------------------
    /** Configures WPI DataLogManager to log to USB if available, and logs DS and NetworkTables data */
    private void configureWpiLogging(){
        if (new java.io.File("/u/").exists()) {
            DataLogManager.start("/u/logs");
        } else if (new java.io.File("/media/sda1/").exists()) {
            DataLogManager.start("/media/sda1/logs");
        } else {
            DataLogManager.start(); // Default location
        }

        // Log DriverStation data (joystick inputs, match time, etc.)
        DriverStation.startDataLog(DataLogManager.getLog());
        
        // Log all NetworkTables data (pose, module states, etc.)
        DataLogManager.logNetworkTables(true);
    }

}

