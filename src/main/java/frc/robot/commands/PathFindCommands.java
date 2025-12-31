// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class PathFindCommands {

  private static final PathConstraints DEFAULT_CONSTRAINTS = Constants.PathFinding.kDefualtConstraints;

  /**
  * Pathfind to a path, follow it, then run an action with custom constraints
  */
  public static Command pathfindAndDo(String pathName, Command scoringAction, PathConstraints constraints) {
      try {
          PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
          return AutoBuilder.pathfindThenFollowPath(path, constraints)
              .andThen(scoringAction);
      } catch (Exception e) {
          return Commands.print("Failed to load path: " + pathName);
      }
  }

  /**
  * Pathfind to a path, follow it, then run an action with default constraints
  */
  public static Command pathfindAndDo(String pathName, Command Action) {
      return pathfindAndDo(pathName, Action, DEFAULT_CONSTRAINTS);
  }

  /**
  * Pathfind directly to a pose, then run an action with custom constraints
  */
  public static Command pathfindToPoseAndDo(Pose2d targetPose, Command action, PathConstraints constraints) {
      return AutoBuilder.pathfindToPose(targetPose, constraints)
          .andThen(action);
  }

  /**
  * Pathfind directly to a pose, then run an action with default constraints
  */
  public static Command pathfindToPoseAndDo(Pose2d targetPose, Command action) {
      return pathfindToPoseAndDo(targetPose, action, DEFAULT_CONSTRAINTS);
  }

  /**
  * Pathfind to path with custom constraints
  */
  public static Command pathfindToPath(String pathName, PathConstraints constraints) {
      try {
          PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
          return AutoBuilder.pathfindThenFollowPath(path, constraints);
      } catch (Exception e) {
          return Commands.print("Failed to load path: " + pathName);
      }
  }

  /**
  * Pathfind to path with default constraints
  */
  public static Command pathfindToPath(String pathName) {
      return pathfindToPath(pathName, DEFAULT_CONSTRAINTS);
  }

}