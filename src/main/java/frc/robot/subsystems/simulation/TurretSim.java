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
  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d turretMech;

  // Visualization constants
  private final double BASE_WIDTH = 60.0;
  private final double BASE_HEIGHT = 10.0;
  private final double turret_LENGTH = 0.3;
  private final double turret_WIDTH = 5;

  /**
   * Creates a new visualization for the turret.
   *
   * @param turretSubsystem The turret subsystem to visualize
   */
  public TurretSim(Turret turretSubsystem) {
    this.turret = turretSubsystem;

    // Create the simulation display
    mech = new Mechanism2d(1, 1);
    root = mech.getRoot("turretRoot", 0.5, 0.5);

    // Add a small base to represent the pivot point
    MechanismLigament2d base = root.append(
      new MechanismLigament2d(
        "Base",
        0, // Small length to represent the pivot
        0,
        2.5, // Small height
        new Color8Bit(Color.kDarkGray)
      )
    );

    // Add the turret mechanism rotating around the base
    turretMech = base.append(
      new MechanismLigament2d(
        "Turret",
        turret_LENGTH,
        0, // Initial angle
        turret_WIDTH,
        new Color8Bit(Color.kRed)
      )
    );

    // Initialize visualization
    SmartDashboard.putData("turret Sim", mech);
  }

  @Override
  public void periodic() {
    // Update turret angle
    double currentAngleRad = turret.getSimulation().getAngleRads();
    turretMech.setAngle(Units.radiansToDegrees(currentAngleRad));

    // Add telemetry data
    SmartDashboard.putNumber(
      "turret Angle (deg)",
      Units.radiansToDegrees(currentAngleRad)
    );
    SmartDashboard.putNumber(
      "turret Velocity (deg/s)",
      Units.radiansToDegrees(turret.getSimulation().getVelocityRadPerSec())
    );
    SmartDashboard.putNumber(
      "turret Current (A)",
      turret.getSimulation().getCurrentDrawAmps()
    );
  }
}
