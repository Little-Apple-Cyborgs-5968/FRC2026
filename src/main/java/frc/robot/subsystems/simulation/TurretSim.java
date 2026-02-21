package frc.robot.subsystems.simulation;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Turret;

/**
 * Visualization for the turret subsystem in simulation.
 */
public class TurretSim extends SubsystemBase {

  

  private final Turret turret;

  // Simulation display
  // private final Mechanism2d mech;
  // private final MechanismRoot2d root;
  // private final MechanismLigament2d turretMech;



  // Visualization constants
  private final double turret_LENGTH = 0.3;
  private final double turret_WIDTH = 5;

  /**
   * Creates a new visualization for the turret.
   *
   * @param turretSubsystem The turret subsystem to visualize
   */
  public TurretSim(Turret turretSubsystem) {
    this.turret = turretSubsystem;
  }

  @Override
  public void periodic() {
    // Update turret angle (convert from rotations to radians for mechanism display)
    double currentAngleRad = turret.getPosition() * 2.0 * Math.PI;

    // Add telemetry data
    SmartDashboard.putNumber(
      "TURRET/turret Angle (deg)",
      Units.radiansToDegrees(currentAngleRad)
    );
    SmartDashboard.putNumber(
      "TURRET/turret Velocity (deg/s)",
      Units.rotationsToDegrees(turret.getVelocity())
    );
    SmartDashboard.putNumber(
      "TURRET/turret Current (A)",
      turret.getCurrent()
    );
    
    // Add target and error telemetry
    SmartDashboard.putNumber(
      "TURRET/Target Angle (deg)",
      turret.getTargetAngleDegrees()
    );
    SmartDashboard.putNumber(
      "TURRET/Target Velocity (deg/s)",
      turret.getTargetVelocityDegPerSec()
    );
    SmartDashboard.putNumber(
      "TURRET/Error (deg)",
      turret.getErrorDegrees()
    );
    
  }
}
