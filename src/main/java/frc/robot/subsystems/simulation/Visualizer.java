// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Turret;


public class Visualizer extends SubsystemBase {
  private final StructPublisher<Pose3d> turretPosePublisher;
  private final Turret turret;
  /** Creates a new Visualizer. */
  public Visualizer(Turret turretSubsystem) {
    this.turret = turretSubsystem;
        // Initialize NetworkTables publisher
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    turretPosePublisher = inst.getStructTopic("VISUALIZER/Turret Pose", Pose3d.struct).publish();
  }

  @Override
  public void periodic() {
    double currentAngleRad = turret.getSimulation().getAngleRads();
            //Publish Pose3d with turret rotation to NetworkTables (FOR FANCY FULL ROBOT VISUALIT)
    Pose3d turretPose = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, currentAngleRad));
    turretPosePublisher.set(turretPose);
  }
}
