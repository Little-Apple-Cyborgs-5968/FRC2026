// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;


/**
 * Handles all SmartDashboard/Shuffleboard visualizations.
 * Call update() periodically to refresh the robot pose.
 */
public class DashboardPublisher {
    private final Field2d m_field = new Field2d();
    private final CommandSwerveDrivetrain m_drivetrain;

    public DashboardPublisher(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        SmartDashboard.putData("Field", m_field);
        
        // Subscribe to PathPlanner's active path
        configurePathPlannerLogging();
    }

    /** Sets up PathPlanner to automatically log paths to our Field2d */
    private void configurePathPlannerLogging() {
        // PathPlanner will call these whenever a path starts/ends
        com.pathplanner.lib.util.PathPlannerLogging.setLogActivePathCallback((poses) -> {
            m_field.getObject("activePath").setPoses(poses);
        });

        // com.pathplanner.lib.util.PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        //     m_field.getObject("targetPose").setPoses(List.of(pose));
        // });

        com.pathplanner.lib.util.PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Optional: shows where PathPlanner thinks the robot is
            // m_field.getObject("ppCurrentPose").setPoses(List.of(pose));
        });
    }

    /** Call this every robot periodic cycle */
    public void update() {
        m_field.setRobotPose(m_drivetrain.getState().Pose);
    }

    /** Display a WPILib trajectory on the field */
    public void showTrajectory(Trajectory trajectory) {
        m_field.getObject("trajectory").setTrajectory(trajectory);
    }

    /** Display a PathPlanner path on the field */
    public void showPathPlannerPath(PathPlannerPath path) {
        m_field.getObject("pathPlannerPath").setPoses(path.getPathPoses());
    }
}
