// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;


public class Visualizer extends SubsystemBase {
  private final StructPublisher<Pose3d> turretPosePublisher;
  private final Turret turret;
  private final Shooter shooter;
  /** Creates a new Visualizer. */
  public Visualizer(Turret turretSubsystem, Shooter shooterSubsystem) {
    this.turret = turretSubsystem;
    this.shooter = shooterSubsystem;
        // Initialize NetworkTables publisher
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    turretPosePublisher = inst.getStructTopic("VISUALIZER/Turret Pose", Pose3d.struct).publish();
  }

  @Override
  public void periodic() {
    double currentAngleRad = turret.getSimulation().getAngleRads();
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
      new Rotation3d(0, hoodAngleRad, currentAngleRad) // Add 180 degrees to match physical orientation
    );
    turretPosePublisher.set(turretPose);
  }
}
