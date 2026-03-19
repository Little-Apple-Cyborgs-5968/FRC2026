// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class Visualizer extends SubsystemBase {
  private final StructPublisher<Pose3d> turretPosePublisher;
  private final StructPublisher<Pose3d> climberPosePublisher;
  private final StructPublisher<Pose2d> targetPosePublisher;
  private final StructArrayPublisher<Pose3d> componentsPublisher;
  private final Turret turret;
  private final Shooter shooter;
  private final Climber climber;
  private final Spindexer spindexer;
  private final Feeder feeder;
  private final CommandSwerveDrivetrain drivetrain;
  
  /** Creates a new Visualizer. */
  public Visualizer(Turret turretSubsystem, Shooter shooterSubsystem, Climber climberSubsystem, Spindexer spindexerSubsystem, Feeder feederSubsystem, CommandSwerveDrivetrain drivetrainSubsystem) {
    this.turret = turretSubsystem;
    this.shooter = shooterSubsystem;
    this.climber = climberSubsystem;
    this.spindexer = spindexerSubsystem;
    this.feeder = feederSubsystem;
    this.drivetrain = drivetrainSubsystem;
        // Initialize NetworkTables publisher
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    turretPosePublisher = inst.getStructTopic("VISUALIZER/Turret Pose", Pose3d.struct).publish();
    climberPosePublisher = inst.getStructTopic("VISUALIZER/Climber Pose", Pose3d.struct).publish();
    targetPosePublisher = inst.getStructTopic("VISUALIZER/Target Pose", Pose2d.struct).publish();
    componentsPublisher = inst.getStructArrayTopic("VISUALIZER/Components Poses", Pose3d.struct).publish();
  }

  @Override
  public void periodic() {
    // Use the motor encoder angle (0 = forward convention) rather than raw sim physics angle,
    // so the visualizer stays correct regardless of the physical starting position.
    double currentAngleRad = turret.getPosition() * 2.0 * Math.PI;
    double hoodAngleDeg = shooter.getHoodAngle();
    double hoodAngleRad = Units.degreesToRadians(hoodAngleDeg);
    
    // Get turret offset from Constants (fixed position on robot)
    double turretOffsetX = Constants.Turret.kTurretOffsetX;
    double turretOffsetY = Constants.Turret.kTurretOffsetY;
    double turretOffsetZ = Constants.Turret.kTurretOffsetZ;
    
    // Publish Pose3d: translation is fixed position, rotation happens at that position
    // The turret rotates in place, it does not move
    Pose3d turretPose = new Pose3d(
      new Translation3d(turretOffsetX, turretOffsetY, turretOffsetZ), 
      new Rotation3d(0, hoodAngleRad, currentAngleRad + Math.PI) // Add 180 degrees to match physical orientation
    );



    double climberX = -0.079; // Adjust as needed to match physical robot
    double climberY = 0.273 ; // Adjust as needed
    double climberBaseZ = 0.15; // Base height of the climber
    double climberZrange = 0.3; // Max extension range of the climber in meters

    double currentClimberHeight = climberZrange * (climber.getPosition() - Constants.Climber.kRetractSetpointRotations) / (Constants.Climber.kExtendSetpointRotations - Constants.Climber.kRetractSetpointRotations);
    
    Pose3d climberPose = new Pose3d(
      new Translation3d(climberX, climberY, climberBaseZ +currentClimberHeight),
      new Rotation3d(0, 0, 0) // No rotation for climber visualization
    );

    turretPosePublisher.set(turretPose);
    climberPosePublisher.set(climberPose);
    componentsPublisher.set(new Pose3d[] {turretPose, climberPose});

    boolean spindexerSpinning = Math.abs(spindexer.getVelocity()) > 0.1 || Math.abs(spindexer.getTargetVelocity()) > 0.1;
    boolean feederSpinning = feeder.isRunning() || Math.abs(feeder.getVelocity()) > 0.1;
    boolean flywheelSpinning = shooter.getTargetFlywheelVelocity() > 0.1 || Math.abs(shooter.getAverageFlywheelVelocity()) > 0.1;
    
    if (spindexerSpinning && feederSpinning && flywheelSpinning) {
      targetPosePublisher.set(shooter.getTargetPose());
    } else {
      targetPosePublisher.set(drivetrain.getState().Pose);
    }
  }
}
